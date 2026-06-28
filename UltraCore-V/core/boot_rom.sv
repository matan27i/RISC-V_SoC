// Boot ROM - RTL
// 2 KB (512 x 32) synchronous ROM, inferred as a true dual-port BRAM:
//
//   Port A - fetch-fill port: serves I-cache line fills (4-word bursts)
//            for the boot region using the same req/data_valid/ready
//            burst protocol as main_memory, with 1-cycle BRAM latency.
//   Port B - AXI4-Lite read-only target on the SoC crossbar, so software
//            can also LOAD from the boot region (constants, tables).
//            Any write attempt is answered with SLVERR (it is a ROM).
//
// The ROM occupies the 8 KB window 0x0000_0000 - 0x0000_1FFF of the SoC
// map; only addr[10:2] selects a word, so the 2 KB image mirrors four
// times inside the window.
//
// Contents: hardcoded bootloader. It initializes nothing UART-side
// (the UART is ready at reset; baud is fixed by hardware parameters),
// polls the UART STATUS register, prints "LIVE\r\n" and jumps to the
// main RAM execution space at 0x0000_2000. Register usage: x5 = UART
// base, x6 = character argument, x1 = link, x8 = scratch.
//
// The image is written by an initial block, which Vivado synthesizes
// into BRAM INIT strings (UG901-supported ROM inference).

module boot_rom (
  input  logic        clk,
  input  logic        resetn,

  // Port A: I-cache line-fill burst interface (main_memory protocol)
  input  logic        fetch_req,         // Held high while a fill is pending
  input  logic [31:0] fetch_addr,        // Line-aligned byte address
  output logic [31:0] fetch_data,        // Burst word output
  output logic        fetch_data_valid,  // Pulses once per word (4 total)
  output logic        fetch_ready,       // Pulses with the last word

  // Port B: AXI4-Lite target, read-only (AxPROT omitted: terminated at
  // the interconnect)
  input  logic [12:0] s_axi_awaddr,
  input  logic        s_axi_awvalid,
  output logic        s_axi_awready,
  input  logic [31:0] s_axi_wdata,
  input  logic [3:0]  s_axi_wstrb,
  input  logic        s_axi_wvalid,
  output logic        s_axi_wready,
  output logic [1:0]  s_axi_bresp,
  output logic        s_axi_bvalid,
  input  logic        s_axi_bready,
  input  logic [12:0] s_axi_araddr,
  input  logic        s_axi_arvalid,
  output logic        s_axi_arready,
  output logic [31:0] s_axi_rdata,
  output logic [1:0]  s_axi_rresp,
  output logic        s_axi_rvalid,
  input  logic        s_axi_rready
);

  localparam int unsigned ROM_WORDS = 512;          // 2 KB
  localparam logic [1:0]  RESP_OKAY   = 2'b00;
  localparam logic [1:0]  RESP_SLVERR = 2'b10;

  
  // Storage + bootloader image
  
  (* rom_style = "block" *)
  logic [31:0] rom [0:ROM_WORDS-1];

  initial begin
    // Fill with NOPs first so unused locations are well defined
    for (int i = 0; i < ROM_WORDS; i++)
      rom[i] = 32'h0000_0013;                       // addi x0, x0, 0

    // Bootloader (hand-assembled RV32I, see comments)
    rom[  0] = 32'h400002B7;   // 0x0000: lui   x5, 0x40000     ; x5 = UART base
    rom[  1] = 32'h04C00313;   // 0x0004: addi  x6, x0, 'L'
    rom[  2] = 32'h034000EF;   // 0x0008: jal   x1, putc
    rom[  3] = 32'h04900313;   // 0x000C: addi  x6, x0, 'I'
    rom[  4] = 32'h02C000EF;   // 0x0010: jal   x1, putc
    rom[  5] = 32'h05600313;   // 0x0014: addi  x6, x0, 'V'
    rom[  6] = 32'h024000EF;   // 0x0018: jal   x1, putc
    rom[  7] = 32'h04500313;   // 0x001C: addi  x6, x0, 'E'
    rom[  8] = 32'h01C000EF;   // 0x0020: jal   x1, putc
    rom[  9] = 32'h00D00313;   // 0x0024: addi  x6, x0, 0x0D    ; '\r'
    rom[ 10] = 32'h014000EF;   // 0x0028: jal   x1, putc
    rom[ 11] = 32'h00A00313;   // 0x002C: addi  x6, x0, 0x0A    ; '\n'
    rom[ 12] = 32'h00C000EF;   // 0x0030: jal   x1, putc
    rom[ 13] = 32'h000023B7;   // 0x0034: lui   x7, 0x2         ; x7 = 0x2000
    rom[ 14] = 32'h00038067;   // 0x0038: jalr  x0, 0(x7)       ; jump to main RAM
    rom[ 15] = 32'h0082A403;   // 0x003C: putc: lw x8, 8(x5)    ; UART STATUS
    rom[ 16] = 32'h00147413;   // 0x0040: andi  x8, x8, 1       ; tx_busy
    rom[ 17] = 32'hFE041CE3;   // 0x0044: bne   x8, x0, putc    ; poll while busy
    rom[ 18] = 32'h0062A023;   // 0x0048: sw    x6, 0(x5)       ; TX_DATA = char
    rom[ 19] = 32'h00008067;   // 0x004C: jalr  x0, 0(x1)       ; return
  end

  
  //  Port A: 4-word line-fill burst engine
  //
  //  Cycle 0..3 issue the four BRAM reads; the registered BRAM output
  //  makes word k valid during cycle k+1, so data_valid covers cycles
  //  1..4 and ready pulses together with the last word - the exact
  //  protocol main_memory uses, which l1_icache already understands.
  
  typedef enum logic {
    ROMF_IDLE  = 1'b0,
    ROMF_BURST = 1'b1
  } romf_state_t;

  romf_state_t fstate;
  logic [8:0]  f_base;      // Word address of the line's first word
  logic [2:0]  f_cyc;       // Burst cycle counter 0..4
  logic [31:0] dout_a;      // Registered BRAM read port A

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      fstate <= ROMF_IDLE;
      f_base <= '0;
      f_cyc  <= '0;
    end
    else begin
      case (fstate)
        ROMF_IDLE: begin
          f_cyc <= '0;
          if (fetch_req) begin
            f_base <= fetch_addr[10:2];   // Line-aligned: bits [3:2] = 00
            fstate <= ROMF_BURST;
          end
        end

        ROMF_BURST: begin
          f_cyc <= f_cyc + 3'd1;
          if (f_cyc == 3'd4)
            fstate <= ROMF_IDLE;
        end

        default: fstate <= ROMF_IDLE;
      endcase
    end
  end

  // BRAM port A: synchronous read, no reset (clean BRAM inference).
  // The read of cycle 4 is harmless garbage that is never presented.
  always_ff @(posedge clk) begin
    dout_a <= rom[f_base + {7'b0, f_cyc[1:0]}];
  end

  assign fetch_data       = dout_a;
  assign fetch_data_valid = (fstate == ROMF_BURST) && (f_cyc >= 3'd1);
  assign fetch_ready      = (fstate == ROMF_BURST) && (f_cyc == 3'd4);

  
  //  Port B: AXI4-Lite read path (one extra WAIT state for the BRAM
  //  read latency); writes are accepted and answered with SLVERR.
  
  typedef enum logic [1:0] {
    RD_IDLE = 2'b00,
    RD_XFER = 2'b01,
    RD_WAIT = 2'b10,
    RD_RESP = 2'b11
  } rd_state_t;

  rd_state_t  rd_state;
  logic [8:0] b_addr;
  logic [31:0] dout_b;

  // BRAM port B: synchronous read, no reset
  always_ff @(posedge clk) begin
    dout_b <= rom[b_addr];
  end

  // dout_b is stable during RD_RESP (b_addr does not change), so it can
  // legally drive RDATA directly while RVALID is high.
  assign s_axi_rdata = dout_b;

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      rd_state      <= RD_IDLE;
      s_axi_arready <= 1'b0;
      s_axi_rvalid  <= 1'b0;
      s_axi_rresp   <= RESP_OKAY;
      b_addr        <= '0;
    end
    else begin
      case (rd_state)
        RD_IDLE: begin
          if (s_axi_arvalid) begin
            s_axi_arready <= 1'b1;
            rd_state      <= RD_XFER;
          end
        end

        RD_XFER: begin
          // AR handshake completes here; ARADDR is still on the bus
          s_axi_arready <= 1'b0;
          b_addr        <= s_axi_araddr[10:2];
          rd_state      <= RD_WAIT;
        end

        RD_WAIT: begin
          // BRAM reads b_addr during this cycle
          rd_state <= RD_RESP;
          s_axi_rvalid <= 1'b1;
          s_axi_rresp  <= RESP_OKAY;
        end

        RD_RESP: begin
          if (s_axi_rready) begin
            s_axi_rvalid <= 1'b0;
            rd_state     <= RD_IDLE;
          end
        end

        default: rd_state <= RD_IDLE;
      endcase
    end
  end

  // Write path: joint AW/W acceptance, SLVERR response (read-only ROM)
  typedef enum logic [1:0] {
    WR_IDLE = 2'b00,
    WR_XFER = 2'b01,
    WR_RESP = 2'b10
  } wr_state_t;

  wr_state_t wr_state;

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      wr_state      <= WR_IDLE;
      s_axi_awready <= 1'b0;
      s_axi_wready  <= 1'b0;
      s_axi_bvalid  <= 1'b0;
      s_axi_bresp   <= RESP_OKAY;
    end
    else begin
      case (wr_state)
        WR_IDLE: begin
          if (s_axi_awvalid && s_axi_wvalid) begin
            s_axi_awready <= 1'b1;
            s_axi_wready  <= 1'b1;
            wr_state      <= WR_XFER;
          end
        end

        WR_XFER: begin
          s_axi_awready <= 1'b0;
          s_axi_wready  <= 1'b0;
          s_axi_bresp   <= RESP_SLVERR;   // ROM: all writes rejected
          s_axi_bvalid  <= 1'b1;
          wr_state      <= WR_RESP;
        end

        WR_RESP: begin
          if (s_axi_bready) begin
            s_axi_bvalid <= 1'b0;
            wr_state     <= WR_IDLE;
          end
        end

        default: wr_state <= WR_IDLE;
      endcase
    end
  end

  // Sink for intentionally unused inputs
  wire _unused_inputs;
  assign _unused_inputs = &{1'b0, s_axi_awaddr,
                            s_axi_wdata, s_axi_wstrb, s_axi_araddr[12:11],
                            s_axi_araddr[1:0], fetch_addr[31:11], fetch_addr[1:0]};

endmodule
