import ALU_pkg::*;

module basic_ALU (
    input alu_in_t in,
    output alu_out_t out
);
    always_comb begin : ALU_COMB
        case (in.op)
            NOT_A: begin
                out.result = ~in.a;
                out.cout = 1'b0;
            end
            NOT_B: begin
                out.result = ~in.b;
                out.cout = 1'b0;
            end
            OR: begin
                out.result = in.a | in.b;
                out.cout = 1'b0;
            end
            AND: begin
                out.result = in.a & in.b;
                out.cout = 1'b0;
            end
            XOR: begin
                out.result = in.a ^ in.b;
                out.cout = 1'b0;
            end
            SRL_A: begin
                out.result = in.a >> 1;
                out.cout = in.a[0];
            end
            SRL_B: begin
                out.result = in.b >> 1;
                out.cout = in.b[0];
            end
            PASS_A: begin
                out.result = in.a;
                out.cout = 1'b0;
            end
            PASS_B: begin
                out.result = in.b;
                out.cout = 1'b0;
            end
            ADD: begin
                {out.cout, out.result} = in.a + in.b + in.cin;
            end
            SUB: begin
                // Standard 2's complement subtraction or borrow implementation. Verify architecture requirements.
                {out.cout, out.result} = in.a - in.b - in.cin;
            end
            default: begin
                out.result = '0;
                out.cout = 1'b0;
            end
        endcase
    end

endmodule