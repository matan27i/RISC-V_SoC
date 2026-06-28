// Machine-mode CSR File + Trap Controller (Zicsr) - RTL
// Implements the minimal RISC-V privileged M-mode state needed for
// interrupt and exception handling on the UltraCore-V core.
//
// Implemented CSRs:
//   0x300 mstatus  - only MIE(3), MPIE(7) modeled; MPP(12:11) reads 11
//   0x304 mie      - MTIE(7), MEIE(11) software-writable
//   0x305 mtvec    - trap vector base (direct mode; low 2 bits ignored)
//   0x340 mscratch - scratch register
//   0x341 mepc     - exception program counter
//   0x342 mcause   - trap cause (MSB=1 for interrupts)
//   0x343 mtval    - trap value (kept 0 here)
//   0x344 mip      - MTIP(7), MEIP(11) are read-only views of the
//                    incoming interrupt lines (cleared at the source)
//   0xF14 mhartid  - reads 0 (single hart)
//   other          - read as 0
//
// Trap model (machine mode only, direct mtvec):
//   On trap_take : mepc<-trap_epc, mcause<-trap_cause,
//                  mstatus.MPIE<-MIE, mstatus.MIE<-0
//   On mret      : mstatus.MIE<-MPIE, mstatus.MPIE<-1
//   irq_req asserts when globally enabled (MIE) and an enabled source
//   is pending (mie & mip). External (MEIP) outranks timer (MTIP).
//
// Single-issue interface: at most one of {trap_take, mret, csr_wen}
// is asserted in a cycle for a given retiring instruction (the core
// guarantees this - a trapped CSR instruction has csr_wen gated off).
// The priority ordering below is therefore only defensive.

module csr_file (
  input  logic        clk,
  input  logic        reset_n,

  // Combinational CSR read (addressed by the CSR instruction in EX)
  input  logic [11:0] csr_raddr,
  output logic [31:0] csr_rdata,

  // CSR write (committed by a retiring CSR instruction)
  input  logic        csr_wen,
  input  logic [11:0] csr_waddr,
  input  logic [31:0] csr_wdata,

  // Trap / return control (from the EX-stage trap logic)
  input  logic        trap_take,    // take interrupt/exception this cycle
  input  logic [31:0] trap_cause,
  input  logic [31:0] trap_epc,
  input  logic        mret,         // mret retiring this cycle

  // Interrupt source lines (level, synchronous to clk)
  input  logic        irq_timer,    // -> mip.MTIP
  input  logic        irq_ecc,      // -> mip.MEIP (external)

  // Outputs consumed by the core
  output logic [31:0] mtvec_o,      // trap vector (redirect target on trap)
  output logic [31:0] mepc_o,       // return address (redirect target on mret)
  output logic        irq_req,      // an enabled interrupt is pending (MIE set)
  output logic [31:0] irq_cause_o,  // cause to latch when irq_req is taken
  output logic        irq_wake      // a locally-enabled IRQ pends (ignores MIE) - WFI wake
);

  
  // State
  
  logic        mstatus_mie, mstatus_mpie;
  logic        mie_mtie, mie_meie;
  logic [31:0] mtvec, mepc, mcause, mscratch, mtval;

  // mip bits are read-only reflections of the incoming lines. The
  // sources are level-held until software clears them at the device
  // (Timer CTRL.CLEAR / ECC CTRL.CLEAR_STATUS), so no storage here.
  logic mip_mtip, mip_meip;
  assign mip_mtip = irq_timer;
  assign mip_meip = irq_ecc;

  
  // Combinational read mux
  
  always_comb begin
    case (csr_raddr)
      12'h300: csr_rdata = {19'b0, 2'b11, 3'b0, mstatus_mpie, 3'b0, mstatus_mie, 3'b0};
      12'h304: csr_rdata = {20'b0, mie_meie, 3'b0, mie_mtie, 7'b0};
      12'h305: csr_rdata = mtvec;
      12'h340: csr_rdata = mscratch;
      12'h341: csr_rdata = mepc;
      12'h342: csr_rdata = mcause;
      12'h343: csr_rdata = mtval;
      12'h344: csr_rdata = {20'b0, mip_meip, 3'b0, mip_mtip, 7'b0};
      12'hF14: csr_rdata = 32'h0;            // mhartid
      default: csr_rdata = 32'h0;
    endcase
  end

  
  // State update. Priority: trap > mret > CSR write (mutually
  // exclusive in practice; see header).
  
  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      mstatus_mie  <= 1'b0;       // interrupts off out of reset
      mstatus_mpie <= 1'b0;
      mie_mtie     <= 1'b0;
      mie_meie     <= 1'b0;
      mtvec        <= '0;
      mepc         <= '0;
      mcause       <= '0;
      mscratch     <= '0;
      mtval        <= '0;
    end
    else if (trap_take) begin
      mepc         <= trap_epc;
      mcause       <= trap_cause;
      mstatus_mpie <= mstatus_mie;
      mstatus_mie  <= 1'b0;       // disable interrupts on trap entry
    end
    else if (mret) begin
      mstatus_mie  <= mstatus_mpie;
      mstatus_mpie <= 1'b1;       // MPIE set to 1 on return (spec)
    end
    else if (csr_wen) begin
      case (csr_waddr)
        12'h300: begin
          mstatus_mie  <= csr_wdata[3];
          mstatus_mpie <= csr_wdata[7];
        end
        12'h304: begin
          mie_mtie <= csr_wdata[7];
          mie_meie <= csr_wdata[11];
        end
        12'h305: mtvec    <= csr_wdata;
        12'h340: mscratch <= csr_wdata;
        12'h341: mepc     <= csr_wdata;
        12'h342: mcause   <= csr_wdata;
        12'h343: mtval    <= csr_wdata;
        default: ;                 // read-only / unimplemented: ignore
      endcase
    end
  end

  
  // Outputs
  
  assign mtvec_o = mtvec;
  assign mepc_o  = mepc;

  logic pend_timer, pend_ext;
  assign pend_timer = mip_mtip & mie_mtie;
  assign pend_ext   = mip_meip & mie_meie;

  assign irq_req     = mstatus_mie & (pend_timer | pend_ext);
  // External interrupt (cause 11) outranks machine timer (cause 7)
  assign irq_cause_o = pend_ext ? 32'h8000_000B : 32'h8000_0007;
  // WFI wakes on any locally-enabled pending interrupt even if MIE=0,
  // so a WFI with global interrupts disabled cannot deadlock.
  assign irq_wake    = pend_timer | pend_ext;

endmodule
