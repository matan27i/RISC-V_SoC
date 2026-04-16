# 💻 UltraCore-V (RISC-V 32 5-stage Pipeline, Dynamic Branch Prediction, L1 Cache)

**UltraCore-V** is a high-performance, 32-bit RISC-V soft-core processor designed from scratch in SystemVerilog. It features a robust 5-stage pipeline and is architected specifically for FPGA efficiency, incorporating advanced microarchitectural features like dynamic branch prediction and a full L1 cache hierarchy.

## 🔑 Key Specifications
- **ISA:** RISC-V RV32I (Base Integer Instruction Set).
- **Pipeline:** 5-Stage (Fetch, Decode, Execute, Memory, Write-Back).
  
## 🛠️ Microarchitectural Features

### 1. Dynamic Branch Prediction (BTB)
- **Implementation:** 64-entry Branch Target Buffer (BTB).
- **Logic:** Uses a **2-bit saturating counter** (Hysteresis) to predict branch outcomes (Strongly/Weakly Taken/Not Taken).
- **FPGA Optimization:** Tailored for **Block RAM (BRAM)** to save logic resources.
- **Latency Solution:** Implements a **Lookahead PC** mechanism in the Fetch stage to hide the 1-cycle BRAM read latency, maintaining a seamless single-cycle fetch throughput.

### 2. Hazard Management & Interlocks
- **Forwarding Unit:** Resolves Read-After-Write (RAW) data hazards by routing ALU results from EX/MEM and MEM/WB stages back to the ALU inputs.
- **Hazard Detection Unit:** Detects Load-Use hazards and automatically stalls the pipeline, injecting "bubbles" to ensure data integrity.
- **Misprediction Recovery:** Hardware-driven synchronous flushes (`IF/ID` and `ID/EX` clearing) when a branch prediction is resolved as incorrect in the Execute stage.

### 3. Memory Subsystem (L1 Caches & Main Memory)
- **I-Cache:** Direct-mapped Level 1 Instruction Cache (1KB) utilizing Lookahead addressing to absorb synchronous BRAM latency.
- **D-Cache:** Direct-mapped Level 1 Data Cache (1KB) featuring a **Write-Back policy** with dirty bits, significantly minimizing slow main memory accesses.
- **Memory Arbiter:** A unified main memory controller featuring a dual-port arbiter to seamlessly resolve and prioritize simultaneous burst transfer requests from both caches.
- **Hardware-Aware Design:** `Valid` and `Dirty` bits are intentionally decoupled from the BRAM arrays and mapped to standard Flip-Flops. This enables immediate, single-cycle synchronous resets and avoids heavy distributed RAM (LUT) inference.

## 📂 Project Structure
* `src/design/`: SystemVerilog RTL source files.
    * `5pipeline_riscv_core.sv`: Top-level core integration.
    * `decode.sv`, `alu.sv`, `branch_control.sv`, `control.sv`: Core execution datapath and control logic.
    * `forwarding_unit.sv`, `hazard_detection.sv`: Hazard resolution units.
    * `register_file.sv`: 32x32-bit CPU registers.
    * `btb.sv`: Branch Target Buffer module.
    * `l1_icache.sv`, `l1_dcache.sv`: Level 1 Instruction and Data Caches.
    * `main_memory.sv`: Memory controllers and backing storage.
* `src/pkg/`: Global packages.
    * `risc_pkg.sv`: Global definitions, structs, and enumerations.

## 🔧 Tools
- **Language:** SystemVerilog, C
- **Simulation:** Questa / Icarus Verilog / Vivado Simulator
- **Synthesis/Implementation:** Precision Synthesis / Xilinx Vivado (Targeting FPGA BRAMs and LUTs)
- **Board Design:** Altium Designer / KiCad

## 🗺️ Roadmap & Development Phases

**Phase 1: Core Design & Validation (Current)**
- [x] 5-Stage Pipeline Core.
- [x] Forwarding & Hazard Detection.
- [x] Dynamic Branch Prediction (BRAM based).
- [x] L1 Instruction Cache & Data Cache (Write-Back).
- [ ] Core-level SystemVerilog Testbench & Waveform Verification.

**Phase 2: System-on-Chip (SoC) Integration**
- [ ] AXI4-Lite standard data bus integration.
- [ ] Peripheral connections (UART for communication, GPIOs, Timers).
- [ ] Custom Hardware Accelerator (e.g., Crypto) integrated as an AXI Slave/Master.

**Phase 3: SoC Level Validation (HW/SW Co-Design)**
- [ ] Write Bare-metal C code to configure Timers, print via UART, and interface with the Hardware Accelerator.
- [ ] Full SoC simulation running the compiled C code to verify hardware-software handshakes.

**Phase 4: PCB Design & Hardware Bring-Up**
- [ ] Schematic capture and PCB Layout (Altium/KiCad) for the FPGA board.
- [ ] Generate Bitstream and program the physical FPGA.
- [ ] Lab testing: Oscilloscope/Logic Analyzer probing and Serial (UART) connection to bring the SoC to life.
- [ ] DFT (Design for Test) - SCAN & BIST insertion for silicon testing readiness.
