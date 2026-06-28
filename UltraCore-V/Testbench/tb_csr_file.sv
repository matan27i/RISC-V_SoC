// Testbench - SIMULATION ONLY
// Unit test for csr_file.sv (CSR read/write, interrupt enable/pending,
// trap entry, and mret).
`timescale 1ns/1ps

module tb_csr_file;
  int errors = 0;
  logic clk = 0; always #5 clk = ~clk;
  logic reset_n = 0;

  logic [11:0] raddr; logic [31:0] rdata;
  logic        wen;   logic [11:0] waddr; logic [31:0] wdata;
  logic        trap_take; logic [31:0] trap_cause, trap_epc; logic mret;
  logic        irq_timer = 0, irq_ecc = 0;
  logic [31:0] mtvec_o, mepc_o, irq_cause_o; logic irq_req;

  csr_file dut (
    .clk(clk), .reset_n(reset_n),
    .csr_raddr(raddr), .csr_rdata(rdata),
    .csr_wen(wen), .csr_waddr(waddr), .csr_wdata(wdata),
    .trap_take(trap_take), .trap_cause(trap_cause), .trap_epc(trap_epc), .mret(mret),
    .irq_timer(irq_timer), .irq_ecc(irq_ecc),
    .mtvec_o(mtvec_o), .mepc_o(mepc_o), .irq_req(irq_req), .irq_cause_o(irq_cause_o),
    .irq_wake()
  );

  task automatic csr_write(input [11:0] a, input [31:0] d);
    @(negedge clk); wen = 1; waddr = a; wdata = d;
    @(negedge clk); wen = 0;
  endtask

  task automatic check(input bit c, input string m);
    if (c) $display("OK   : %s", m);
    else begin $display("ERROR: %s", m); errors++; end
  endtask

  initial begin
    wen = 0; trap_take = 0; mret = 0; raddr = 0; waddr = 0; wdata = 0; trap_cause = 0; trap_epc = 0;
    repeat (3) @(negedge clk); reset_n = 1; @(negedge clk);

    // 1. Out of reset: interrupts globally disabled
    irq_timer = 1;
    @(negedge clk);
    check(irq_req === 1'b0, "irq_req low out of reset (MIE=0) despite pending timer");

    // 2. mtvec write/read
    csr_write(12'h305, 32'h0000_2100);
    raddr = 12'h305; #1;
    check(rdata === 32'h0000_2100, "mtvec readback");
    check(mtvec_o === 32'h0000_2100, "mtvec_o exposed");

    // 3. Enable timer interrupt: mie.MTIE then mstatus.MIE
    csr_write(12'h304, 32'h0000_0080);     // MTIE (bit7)
    raddr = 12'h304; #1;
    check(rdata === 32'h0000_0080, "mie.MTIE readback");
    csr_write(12'h300, 32'h0000_0008);     // mstatus.MIE (bit3)
    @(negedge clk);
    check(irq_req === 1'b1, "irq_req high once MIE+MTIE set and timer pending");
    check(irq_cause_o === 32'h8000_0007, "cause = machine timer (0x80000007)");

    // 4. External (ECC) interrupt outranks timer
    csr_write(12'h304, 32'h0000_0880);     // MTIE + MEIE (bits 7,11)
    irq_ecc = 1;
    @(negedge clk);
    check(irq_cause_o === 32'h8000_000B, "cause = external when ECC pending (0x8000000B)");

    // mip reflects the live lines
    raddr = 12'h344; #1;
    check(rdata === 32'h0000_0880, "mip shows MTIP+MEIP from live lines");

    // 5. Take a trap: epc/cause latched, MIE cleared, MPIE = old MIE(1)
    @(negedge clk); trap_take = 1; trap_cause = 32'h8000_000B; trap_epc = 32'h0000_2040;
    @(negedge clk); trap_take = 0;
    raddr = 12'h341; #1; check(rdata === 32'h0000_2040, "mepc latched on trap");
    raddr = 12'h342; #1; check(rdata === 32'h8000_000B, "mcause latched on trap");
    check(mepc_o === 32'h0000_2040, "mepc_o exposed for mret");
    check(irq_req === 1'b0, "irq_req low after trap entry (MIE cleared)");
    raddr = 12'h300; #1;
    check(rdata[3] === 1'b0 && rdata[7] === 1'b1, "mstatus: MIE=0, MPIE=1 after trap");

    // 6. mret restores MIE from MPIE, sets MPIE=1
    @(negedge clk); mret = 1;
    @(negedge clk); mret = 0;
    raddr = 12'h300; #1;
    check(rdata[3] === 1'b1 && rdata[7] === 1'b1, "mstatus: MIE restored to 1 after mret");
    @(negedge clk);
    check(irq_req === 1'b1, "irq_req high again after mret re-enables interrupts");

    // 7. mscratch scratch behavior
    csr_write(12'h340, 32'hDEAD_BEEF);
    raddr = 12'h340; #1;
    check(rdata === 32'hDEAD_BEEF, "mscratch read/write");

    if (errors == 0) $display("==== ALL TESTS PASSED ====");
    else             $display("==== %0d TEST(S) FAILED ====", errors);
    $finish;
  end
endmodule
