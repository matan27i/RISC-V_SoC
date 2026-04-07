module comparator4bit_behavior (
    input  [3:0] a, b,
    output reg equal, greater, less
);

    always @(*) begin
        equal = (a == b);
        greater = (a > b);
        less = (a < b);
    end

endmodule