# 🧠 Multicycle Processor Implementation

## 📌 Overview

This project aims to design a specialized **16-bit multicycle RISC processor** using Verilog. The processor is structured around a **5-stage architecture** consisting of:

- Instruction Fetch
- Instruction Decode
- Execute (ALU)
- Memory Access
- Write Back

It supports a simplified but functional **Instruction Set Architecture (ISA)** with **four instruction types**:  
**R-type**, **I-type**, **J-type**, and **S-type** — enabling arithmetic, logic, memory operations, and conditional branching.

---

## 🧱 Key Features

- **Multicycle design** with clear separation of stages
- **16-bit word and instruction size**
- **8 general-purpose registers** (R0–R7), with R0 hardwired to 0
- **Byte-addressable memory** with **little-endian** ordering
- **ISA** supports:
  - Arithmetic: `ADD`, `SUB`, `ADDI`
  - Logic: `AND`, `ANDI`
  - Memory: `LW`, `SW`, `LBu`, `LBs`
  - Control flow: `BEQ`, `BGT`, `JMP`, `CALL`, `RET`

---
