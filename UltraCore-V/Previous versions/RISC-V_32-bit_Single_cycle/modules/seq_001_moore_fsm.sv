module seq_001_moore_fsm (
    output logic det,
    input  logic inp,
    input  logic clk,
    input  logic rst_n
);

    // Moore machine requires 4 states to detect a 3-bit sequence
    typedef enum logic [1:0] {
        S_IDLE = 2'b00,
        S_0    = 2'b01,
        S_00   = 2'b10,
        S_001  = 2'b11
    } state_t;

    state_t state_q, next_state;

    // Sequential logic: State register
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state_q <= S_IDLE;
        else
            state_q <= next_state;
    end

    // Combinational logic: Next state
    always_comb begin
        next_state = state_q; // Default assignment to prevent latches

        case (state_q)
            S_IDLE: begin
                if (!inp) next_state = S_0;
                else      next_state = S_IDLE;
            end
            S_0: begin
                if (!inp) next_state = S_00;
                else      next_state = S_IDLE;
            end
            S_00: begin
                if (!inp) next_state = S_00;
                else      next_state = S_001;
            end
            S_001: begin
                // Transitioning out of the detection state
                if (!inp) next_state = S_0;
                else      next_state = S_IDLE;
            end
            default: next_state = S_IDLE;
        endcase
    end

    // Output logic: Depends ONLY on the current state (Moore)
    always_comb begin
        if (state_q == S_001)
            det = 1'b1;
        else
            det = 1'b0;
    end

endmodule