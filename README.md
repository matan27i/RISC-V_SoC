# UltraCore-V (RV32I-5P-DBP-L1C)

**UltraCore-V** is a high-performance, 32-bit RISC-V soft-core processor designed from scratch in SystemVerilog. It features a robust 5-stage pipeline and is architected specifically for FPGA efficiency, incorporating advanced microarchitectural features like dynamic branch prediction and an L1 instruction cache.

## Key Specifications
- **ISA:** RISC-V RV32I (Base Integer Instruction Set).
- **Pipeline:** 5-Stage (Fetch, Decode, Execute, Memory, Write-Back).
- 
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

### 3. Memory Subsystem (L1 Cache)
- **I-Cache:** Direct-mapped Level 1 Instruction Cache to reduce average memory access time.
- **Integration:** Integrated Stall FSM to interface with slower external main memory.

## 📂 Project Structure
* `src/RTL_core/`: SystemVerilog source files.
    * `btb.sv`: Branch Target Buffer module.
    * `fetch.sv`: IF stage with Lookahead PC logic.
    * `5pipeline_riscv_core.sv`: Top-level core integration.
    * `alu.sv`, `decode.sv`, `control.sv`: Core execution logic.
* `src/pkg/`: `risc_pkg.sv` containing global definitions.
* `sim/`: Testbenches and simulation scripts.

## 🔧 Tools
- **Language:** SystemVerilog
- **Simulation:** Icarus Verilog / Vivado Simulator
- **Waveform Analysis:** Questa
- **Synthesis/Implementation:** Xilinx Vivado (Targeting FPGA BRAMs and LUTs)

## 🗺️ Roadmap
- [x] 5-Stage Pipeline Core.
- [x] Forwarding & Hazard Detection.
- [x] Dynamic Branch Prediction (BRAM based).
- [ ] L1 Instruction Cache.
- [ ] L1 Data Cache.
- [ ] AXI4-Lite Bus Interface.
- [ ] DFT (Design for Test) - SCAN & BIST insertion.
