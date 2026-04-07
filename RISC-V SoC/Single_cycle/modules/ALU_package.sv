package ALU_pkg;

    // Define the architectural data width.
    parameter XLEN = 32;

    typedef enum logic [3:0] { 
        NOT_A, NOT_B, OR, AND, XOR, SRL_A, SRL_B, PASS_A, PASS_B, ADD, SUB
    } alu_op_t;
      
    typedef struct packed {  // ALU input signals
        logic [XLEN-1:0] a;
        logic [XLEN-1:0] b;
        logic            cin;
        alu_op_t         op;
    } alu_in_t;

    typedef struct packed { // ALU output signals
        logic [XLEN-1:0] result;
        logic            cout;    
    } alu_out_t;

endpackage