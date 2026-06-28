# UltraCore-V SoC — Technical Datasheet

**A 32-bit RISC-V (RV32I + Zicsr) System-on-Chip for Xilinx Artix-7**

| | |
|---|---|
| Core | 5-stage in-order RV32I pipeline, machine-mode Zicsr + trap unit |
| Caches | L1 I-cache and L1 D-cache (direct-mapped, BRAM-backed) |
| Branch prediction | 64-entry BTB (2-bit saturating counters) |
| Interconnect | AXI4-Lite, 1 master → 5 slaves, with a bus timeout watchdog |
| Peripherals | UART (8N1), 32-bit Timer, SEC-DED ECC accelerator, 8-bit GPIO |
| Boot | 8 KB Boot ROM with on-chip bootloader |
| Target clock | 100 MHz (10 ns) |
| Reset | Active-low `resetn`, asynchronous assert / synchronous release |

---

## 1. Block Diagram

```
                          UltraCore-V SoC
  ┌──────────────────────────────────────────────────────────────────┐
  │                                                                    │
  │   ┌───────────────────────────────────────────┐                   │
  │   │   five_stage_pipeline_riscv_core           │                   │
  │   │   IF → ID → EX → MEM → WB                  │   irq_timer ◄──┐  │
  │   │   ┌─────────┐ ┌─────────┐ ┌────────────┐   │   irq_ecc   ◄┐ │  │
  │   │   │ L1 I$   │ │ L1 D$   │ │ csr_file   │   │              │ │  │
  │   │   └────┬────┘ └────┬────┘ │ (Zicsr +   │   │              │ │  │
  │   │        │           │      │  trap/IRQ) │   │              │ │  │
  │   │   ┌────┴───────────┴──┐   └────────────┘   │              │ │  │
  │   │   │   main_memory     │      MMIO port     │              │ │  │
  │   │   │   (16 KB RAM)     │          │         │              │ │  │
  │   │   └───────────────────┘          │         │              │ │  │
  │   └──────────┬────────────────────── │ ────────┘              │ │  │
  │     ROM fill │ (I$ < 0x2000)         │ MMIO (≥0x4000_0000      │ │  │
  │              │                       │       or  <0x2000)      │ │  │
  │              │              ┌────────┴─────────┐               │ │  │
  │              │              │ axi_master_bridge│               │ │  │
  │              │              └────────┬─────────┘               │ │  │
  │              │              ┌────────┴─────────┐               │ │  │
  │              │              │ axi4lite_timeout │ (watchdog)    │ │  │
  │              │              │   _monitor       │──► bus_timeout│ │  │
  │              │              └────────┬─────────┘               │ │  │
  │              │              ┌────────┴─────────┐               │ │  │
  │              │              │   axi_crossbar   │ (addr decode) │ │  │
  │              │              └─┬───┬───┬───┬───┬┘               │ │  │
  │         ┌────┴────┐    ┌──────┘   │   │   │   └──────┐         │ │  │
  │         │boot_rom │◄───┤  ┌───────┘   │   └──────┐   │         │ │  │
  │         │(S0,8KB) │ AXI│  │UART (S1)  │ECC  (S3) │GPIO(S4)     │ │  │
  │         └─────────┘    │  │uart_axi   │ecc_accel │gpio_axi     │ │  │
  │                        │  │ _wrapper  │   ▲      │   │         │ │  │
  │                        │  └────┬──────┘   │      │   └► gpio[7:0]│  │
  │                        │       │ rx tap   │      └──────────────┘│  │
  │                        │  ┌────┴──────┐ ┌─┴──────────┐  irq_ecc ─┘  │
  │                        │  │ uart 8N1  │ │ecc_frame_  │              │
  │                        │  │ tx/rx     │ │ packer→FIFO│              │
  │                        │  └─► txd/rxd │ └────────────┘              │
  │                  Timer (S2)                                         │
  │                  timer_axi_wrapper ──────────────────► irq_timer ───┘
  │                  (32-bit up-counter)                                │
  └──────────────────────────────────────────────────────────────────┘
```

---

## 2. System Memory Map

The map is enforced by **two** decoders: the core's MEM-stage MMIO split and
the crossbar's peripheral decode.

| Region | Base | End | Size | Path | Module |
|---|---|---|---|---|---|
| Boot ROM | `0x0000_0000` | `0x0000_1FFF` | 8 KB | I$ fill (fetch) + AXI (data) | `boot_rom.sv` |
| Main RAM | `0x0000_2000` | `0x3FFF_FFFF`¹ | 16 KB² | Cacheable, via L1 D-cache | `main_memory.sv` |
| UART | `0x4000_0000` | `0x4000_000F` | 16 B | AXI4-Lite | `uart_axi_wrapper.sv` |
| Timer | `0x4000_0010` | `0x4000_001F` | 16 B | AXI4-Lite | `timer_axi_wrapper.sv` |
| ECC accelerator | `0x4000_0020` | `0x4000_002F` | 16 B | AXI4-Lite | `ecc_accel.sv` |
| GPIO | `0x4000_0030` | `0x4000_003F` | 16 B | AXI4-Lite | `gpio_axi.sv` |
| *(unmapped)* | — | — | — | Crossbar → **DECERR** | — |

¹ Logical cacheable window; the physical backing store is 16 KB and aliases.
² Application image is linked at `0x0000_2000` (the boot ROM jumps here).

**Routing rule.** A data access is **MMIO** (uncached, AXI) when its address is
`< 0x0000_2000` (ROM data window) **or** `≥ 0x4000_0000` (peripherals);
everything else is **cacheable RAM**. Instruction fetch below `0x2000` is served
by the Boot ROM fill port, above it by main RAM. The crossbar decodes
`addr[31:4]` to select the peripheral.

---

## 3. Peripheral Register Maps

All peripherals are AXI4-Lite slaves, 32-bit data. Byte offsets are relative to
each peripheral's base address. Unused upper data bits read 0; writes to
read-only registers are accepted with an OKAY response and ignored.

### 3.1 UART — base `0x4000_0000`

| Offset | Name | Access | Bits | Description |
|---|---|---|---|---|
| `0x00` | `TX_DATA` | WO | `[7:0]` | Byte to transmit; accepted only while `TX_BUSY=0` |
| `0x04` | `RX_DATA` | RO | `[7:0]` | Last received byte; **reading clears** RX_VALID/FRAME_ERR/OVERRUN |
| `0x08` | `STATUS` | RO | `[0]` TX_BUSY, `[1]` RX_VALID, `[2]` FRAME_ERR, `[3]` OVERRUN | |
| `0x0C` | — | — | reserved | |

Format 8N1, parameterized `CLK_FREQ_HZ` / `BAUD_RATE` (default 115200), 16×
oversampling RX. The RX byte stream is also tapped internally to feed the ECC
front-end.

### 3.2 Timer — base `0x4000_0010`

| Offset | Name | Access | Bits | Description |
|---|---|---|---|---|
| `0x10` | `CTRL` | RW | `[0]` ENABLE, `[1]` CLEAR (W1, self-clearing) | CLEAR zeroes counter **and** acks the interrupt |
| `0x14` | `COMPARE` | RW | `[31:0]` | Match threshold (resets to `0xFFFF_FFFF`) |
| `0x18` | `COUNT` | RO | `[31:0]` | Live counter value |
| `0x1C` | `STATUS` | RO | `[0]` IRQ_PENDING | Sticky compare-match flag |

A match raises a **sticky** interrupt (`irq_timer`) every `COMPARE+1` enabled
cycles; software acks via `CTRL.CLEAR`.

### 3.3 ECC Accelerator (SEC-DED) — base `0x4000_0020`

| Offset | Name | Access | Bits | Description |
|---|---|---|---|---|
| `0x20` | `DATA` | RO | `[7:0]` | Corrected 8-bit payload |
| `0x24` | `STATUS` | RO | `[0]` DATA_READY, `[1]` SINGLE_ERROR_CORRECTED, `[2]` DOUBLE_ERROR_DETECTED | |
| `0x28` | `CTRL` | RW | `[0]` IRQ_EN, `[1]` CLEAR_STATUS (W1) | |

(13,8) extended-Hamming decoder: zero-latency combinational syndrome decode,
corrects single-bit errors, flags uncorrectable double-bit errors. Raises
`irq_ecc` when `DATA_READY & IRQ_EN`.

### 3.4 GPIO — base `0x4000_0030`

| Offset | Name | Access | Bits | Description |
|---|---|---|---|---|
| `0x30` | `DATA` | RW | `[7:0]` | Read = synchronized pin inputs; Write = output register |
| `0x34` | `DIR` | RW | `[7:0]` | Per-pin direction, 1 = output (drives pin), 0 = input (Hi-Z) |

Tri-state IOBUFs are instantiated at the SoC boundary; all pins reset to inputs.

---

## 4. Machine-Mode CSRs (Zicsr) — `csr_file.sv`

| Addr | Name | Implemented fields |
|---|---|---|
| `0x300` | `mstatus` | MIE `[3]`, MPIE `[7]`; MPP `[12:11]` reads `11` |
| `0x304` | `mie` | MTIE `[7]`, MEIE `[11]` |
| `0x305` | `mtvec` | Trap vector base (direct mode) |
| `0x340` | `mscratch` | Scratch register |
| `0x341` | `mepc` | Exception PC (saved trap address) |
| `0x342` | `mcause` | Trap cause (MSB=1 ⇒ interrupt) |
| `0x343` | `mtval` | Trap value (0) |
| `0x344` | `mip` | MTIP `[7]`, MEIP `[11]` — read-only views of the IRQ lines |
| `0xF14` | `mhartid` | 0 |

**Instructions added:** `CSRRW/CSRRS/CSRRC` (+ immediate forms), `ECALL`,
`EBREAK`, `MRET`. CSR instructions fully serialize (single-cycle redirect to
PC+4) so side effects are globally ordered.

---

## 5. Interrupt & Trap Architecture

| Source | mip bit | Cause code | Priority |
|---|---|---|---|
| ECC accelerator (`irq_ecc`) | MEIP (11) | `0x8000_000B` (machine external) | Higher |
| Timer (`irq_timer`) | MTIP (7) | `0x8000_0007` (machine timer) | Lower |
| `ECALL` (M-mode) | — | `0x0000_000B` (cause 11) | synchronous |
| `EBREAK` | — | `0x0000_0003` (cause 3) | synchronous |

**Trap entry** (taken in EX on a valid instruction when
`mstatus.MIE & (mie & mip)`): `mepc ← trapped PC`, `mcause ← cause`,
`mstatus.MPIE ← MIE`, `mstatus.MIE ← 0`, redirect to `mtvec`. The trapped
instruction and everything younger are flushed (precise — it re-executes after
return).
**Trap return** (`MRET`): `mstatus.MIE ← MPIE`, `MPIE ← 1`, redirect to `mepc`.

Interrupts are level-sourced and held at the device (Timer `CTRL.CLEAR`, ECC
`CTRL.CLEAR_STATUS`) — the handler must ack the source to deassert the line.

---

## 6. Boot Flow

1. Reset → PC = `0x0000_0000` (Boot ROM).
2. Bootloader polls UART `STATUS.TX_BUSY` and prints `LIVE\r\n`.
3. Jumps (`jalr`) to `0x0000_2000` — the main RAM execution space.
4. Application (linked at `0x2000`) runs from cacheable RAM.

---

## 7. AXI4-Lite Bus Watchdog

`axi4lite_timeout_monitor.sv` sits transparently on the master→interconnect
link. If a slave fails to accept a request within `BUS_TIMEOUT_CYCLES`
(default 256), it closes the master handshake and injects `SLVERR`, releasing
the CPU's MMIO stall instead of hanging the pipeline. `bus_timeout` pulses on
each such event.

---

## 8. Top-Level Interface (`soc_top.sv`)

| Port | Dir | Description |
|---|---|---|
| `clk` | in | System clock (100 MHz) |
| `resetn` | in | Active-low reset |
| `uart_txd` | out | UART serial out |
| `uart_rxd` | in | UART serial in |
| `gpio[7:0]` | inout | Tri-state GPIO pads |
| `debug_pc[31:0]` | out | Architectural fetch PC (debug) |
| `bus_timeout` | out | AXI watchdog event pulse (debug) |

Parameters: `CLK_FREQ_HZ` (100 MHz), `BAUD_RATE` (115200),
`BUS_TIMEOUT_CYCLES` (256), `MEM_INIT_FILE` (RAM image).

---

## 9. Programming Example — timer interrupt

```asm
        lui   x5, 0x40000          # peripheral base
        la    x6, handler          # (lui+addi)
        csrrw x0, mtvec, x6        # mtvec = handler
        addi  x7, x0, 0x80
        csrrs x0, mie, x7          # mie.MTIE = 1
        addi  x7, x0, 0x8
        csrrs x0, mstatus, x7      # mstatus.MIE = 1  (global enable)
        addi  x7, x0, 50
        sw    x7, 0x14(x5)         # TIMER COMPARE = 50
        addi  x7, x0, 1
        sw    x7, 0x10(x5)         # TIMER CTRL.ENABLE = 1
loop:   jal   x0, loop             # work; interrupt arrives asynchronously
handler:
        # ... service ...
        addi  x8, x0, 2
        sw    x8, 0x10(x5)         # CTRL.CLEAR = 1  (ack timer)
        mret                       # return to loop
```

---

## 10. Verification Status

All blocks are verified with self-checking testbenches under Icarus Verilog:

| Testbench | Coverage | Result |
|---|---|---|
| `tb_soc_top` | Boot banner, RAM exec, GPIO, timer IRQ line, UART→ECC→GPIO, watchdog transparency | PASS |
| `tb_trap_top` | End-to-end timer interrupt vectoring (mepc/mcause precise, MRET return) | PASS |
| `tb_csr_file` | CSR read/write, enable/pending, trap entry, MRET | PASS |
| `tb_axi4lite_timeout_monitor` | Pass-through, read/write timeout, counter reset, recovery | PASS |
| `tb_uart_timer` | UART loopback + framing errors, timer compare/period | PASS |
| `tb_ecc_accel` | Exhaustive single/double-bit error injection, AXI, IRQ | PASS |

---

## 11. Source File Index

| File | Role |
|---|---|
| `risc_pkg.sv` | Shared types/enums |
| `five_stage_pipeline_riscv_core.sv` | Pipeline core (incl. MMIO/ROM/CSR integration) |
| `csr_file.sv` | Zicsr CSRs + trap controller |
| `decode.sv`, `control.sv`, `alu.sv`, `register_file.sv` | Core datapath/decode |
| `branch_control.sv`, `hazard_detection.sv`, `btb.sv` | Control/prediction |
| `l1_icache.sv`, `l1_dcache.sv`, `main_memory.sv` | Memory hierarchy |
| `axi_master_bridge.sv`, `axi_crossbar.sv`, `axi4lite_timeout_monitor.sv` | Interconnect |
| `boot_rom.sv` | Boot ROM + bootloader |
| `uart*.sv`, `timer*.sv`, `ecc_accel.sv`, `ecc_frame_packer.sv`, `gpio_axi.sv` | Peripherals |
| `soc_top.sv` | Top-level integration |
```
