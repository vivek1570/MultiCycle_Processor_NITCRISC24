# Multi-Cycle Processor - NITC-RISC24

This project implements a **multi-cycle processor** based on the **NITC-RISC24** architecture. It is a 16-bit Little-Endian processor with 8 general-purpose registers. The project simulates the execution of basic RISC instructions through Verilog code, using the **R**, **I**, and **J** instruction formats.

## Architecture Overview

- **16-bit processor**
- **8 general-purpose registers** (R0-R7)
  - R0 is reserved for the **Program Counter (PC)**.
- **Condition Code Register** with two flags:
  - **Carry (C)**
  - **Zero (Z)**

### Instruction Formats:

1. **R-Type (Register Type)**:

   - Format: `OP RA RB RC Unused Condition (CZ)`
   - Example: `ADD`, `ADC`, `NDU`, `NDZ`

2. **I-Type (Immediate Type)**:

   - Format: `OP RA RB Immediate`
   - Example: `LW`, `SW`, `BEQ`

3. **J-Type (Jump Type)**:
   - Format: `OP RA Immediate`
   - Example: `JAL`

## Supported Instructions

- **ADD**: Adds contents of registers `RA` and `RB` and stores the result in `RC`.
- **ADC**: Adds contents of registers `RA` and `RB`, storing the result in `RC` if the carry flag is set.
- **NDU**: Performs a NAND operation between `RA` and `RB`, storing the result in `RC`.
- **LW**: Loads a value from memory into `RA`.
- **SW**: Stores the contents of `RA` into memory.
- **BEQ**: Branches to a new address if the contents of `RA` and `RB` are equal.
- **JAL**: Jumps to an address and stores the next instruction address in `RA`.

## Files

- `mips.v`: The main Verilog file for the multi-cycle processor.
- `control.v`: Control unit module.
- `alucontrol.v`: ALU control logic.
- `datapath.v`: Datapath implementation of the processor.
- `mem.dat`: Memory initialization file for instruction and data memory.
- `tb_mips.v`: Testbench for simulating the processor.

## How to Run

1. Ensure that you have a Verilog simulator (like ModelSim, Verilator, or Icarus Verilog) installed.
2. Compile and run the `tb_mips.v` testbench to simulate the processor.
3. The memory contents for simulation are preloaded from `mem.dat` using the `$readmemh` Verilog function.

## References

- Multi-Cycle Processor Design Assignment (NITC-RISC24)
- Verilog HDL for Simulation and Hardware Synthesis
