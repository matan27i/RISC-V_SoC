module fa_behavioral
(
    input a, b, cin,
    output reg s, c 
);

    always @(*) 
    begin
        {c, s} = a + b + cin; 
    end

endmodule