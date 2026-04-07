module d_ff_behavior_async_hresrt (
    input clk,d,reset,
    output reg q
);
    always @(posedge clk or posedge reset) begin
        if (reset)
            q <= 1'b0;
        else
            q <= d; 
    end
endmodule