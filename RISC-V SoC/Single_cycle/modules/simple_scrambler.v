module simple_scrambler #(
    parameter WIDTH = 8
)(
    input wire [WIDTH-1:0] data_in,
    input wire [WIDTH-1:0] key,
    output wire [WIDTH-1:0] data_out
);

    genvar i;
    
    generate
        for (i = 0; i < WIDTH; i = i + 1) begin : gen_xor_gates
            // Instantiating a 2-input XOR gate for each bit of the bus.
            // The synthesis tool unrolls this loop and maps each operation directly to LUTs.
            assign data_out[i] = data_in[i] ^ key[i];
        end
    endgenerate

endmodule