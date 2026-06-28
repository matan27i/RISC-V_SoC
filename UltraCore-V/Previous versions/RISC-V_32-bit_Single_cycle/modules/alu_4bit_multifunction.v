module alu_4bit_multifunction (
    input      [3:0] a, b, opcode,
    output reg [3:0] x, y
);

    always @(*) begin
        x = 4'b0000;
        y = 4'b0000;

        case (opcode)
            4'b0000: x[0]      = |a;          // Reduction OR
            4'b0001: x[0]      = &a;          // Reduction AND
            4'b0010: x[0]      = ^a;          // Reduction XOR
            4'b0011: x         = a & b;       // Bitwise AND
            4'b0100: x         = a | b;       // Bitwise OR
            4'b0101: x         = a ^ b;       // Bitwise XOR
            4'b0110: x[0]      = (a > b);     // Greater than
            4'b0111: x[0]      = (a < b);     // Less than
            4'b1000: x[0]      = !a;          // Logical NOT
            4'b1001: x[0]      = (a == b);    // Equality
            4'b1010: {y[0], x} = a + b;       // Addition (y[0] holds the Carry Out)
            4'b1011: x         = a - b;       // Subtraction
            4'b1100: {y, x}    = a * b;       // Multiplication (8-bit result total)
            4'b1101: {y, x}    = a >> b;      // Logical Shift Right
            4'b1110: {y, x}    = a << b;      // Logical Shift Left
            4'b1111: x         = ~a;          // Bitwise NOT
            
            default: begin
                $display("Error: Invalid Opcode");
            end
        endcase
    end

endmodule