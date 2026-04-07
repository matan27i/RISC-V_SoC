module barrel_shifter_8bit (
    input      [7:0] In,
    input      [2:0] n,
    input            Lr,
    output reg [7:0] Out
);

    // Behavioral description of a logical barrel shifter
    always @(*) begin
        if (Lr) begin
            Out = In << n;  // Shift Left
        end else begin
            Out = In >> n;  // Shift Right
        end
    end

endmodule