// Testbench - SIMULATION ONLY
// Directed test of muldiv_unit including RV32M corner cases.
`timescale 1ns/1ps
module tb_muldiv_unit;
  int errors = 0;
  logic clk = 0; always #5 clk = ~clk;
  logic reset_n = 0;

  logic        req = 0, advance = 1;
  logic [2:0]  funct3;
  logic [31:0] op_a, op_b, result;
  logic        busy, done;

  muldiv_unit #(.MUL_LATENCY(2)) dut (
    .clk(clk), .reset_n(reset_n), .req(req), .advance(advance),
    .funct3(funct3), .op_a(op_a), .op_b(op_b),
    .result(result), .busy(busy), .done(done));

  task automatic run(input [2:0] f, input [31:0] a, input [31:0] b,
                     input [31:0] exp, input string name);
    int guard = 0;
    @(negedge clk); funct3 = f; op_a = a; op_b = b; req = 1;
    // wait for done
    do begin @(negedge clk); guard++; end while (!done && guard < 100);
    if (result === exp)
      $display("OK   : %s  f=%0d a=%08h b=%08h -> %08h", name, f, a, b, result);
    else begin
      $display("ERROR: %s  f=%0d a=%08h b=%08h -> %08h (exp %08h)", name, f, a, b, result, exp);
      errors++;
    end
    req = 0; @(negedge clk);   // advance=1 so unit returns to IDLE
  endtask

  initial begin
    repeat (3) @(negedge clk); reset_n = 1; @(negedge clk);

    // MUL family
    run(3'b000, 32'd7, 32'd6, 32'd42, "MUL small");
    run(3'b000, 32'hFFFFFFFF, 32'hFFFFFFFF, 32'h00000001, "MUL -1*-1 low");
    run(3'b001, 32'hFFFFFFFF, 32'hFFFFFFFF, 32'h00000000, "MULH -1*-1");
    run(3'b011, 32'hFFFFFFFF, 32'hFFFFFFFF, 32'hFFFFFFFE, "MULHU max*max");
    run(3'b001, 32'h7FFFFFFF, 32'h7FFFFFFF, 32'h3FFFFFFF, "MULH maxpos^2");
    run(3'b010, 32'hFFFFFFFF, 32'h00000002, 32'hFFFFFFFF, "MULHSU -1*2");

    // DIV family
    run(3'b100, 32'd100, 32'd7, 32'd14, "DIV 100/7");
    run(3'b110, 32'd100, 32'd7, 32'd2,  "REM 100%7");
    run(3'b100, -32'sd100, 32'd7, -32'sd14, "DIV -100/7");
    run(3'b110, -32'sd100, 32'd7, -32'sd2,  "REM -100%7");
    run(3'b100, -32'sd100, -32'sd7, 32'd14, "DIV -100/-7");
    run(3'b101, 32'd100, 32'd7, 32'd14, "DIVU 100/7");
    run(3'b111, 32'd100, 32'd7, 32'd2,  "REMU 100%7");

    // Corner cases (spec-defined)
    run(3'b100, 32'd123, 32'd0, 32'hFFFFFFFF, "DIV by zero -> -1");
    run(3'b110, 32'd123, 32'd0, 32'd123,      "REM by zero -> dividend");
    run(3'b101, 32'd123, 32'd0, 32'hFFFFFFFF, "DIVU by zero -> all ones");
    run(3'b111, 32'd123, 32'd0, 32'd123,      "REMU by zero -> dividend");
    run(3'b100, 32'h80000000, 32'hFFFFFFFF, 32'h80000000, "DIV overflow -> INT_MIN");
    run(3'b110, 32'h80000000, 32'hFFFFFFFF, 32'd0,         "REM overflow -> 0");

    if (errors == 0) $display("==== ALL TESTS PASSED ====");
    else             $display("==== %0d TEST(S) FAILED ====", errors);
    $finish;
  end
endmodule
