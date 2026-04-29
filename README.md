# Quasar

A research NPU co-processor architecture for LLM inference acceleration,
designed to accelerate the [Axiom](link) inference engine.

## Architecture

- Weight-stationary 4×4 systolic array (INT8 MAC units)
- 3-stage RISC-V host controller with custom NPU ISA extension
- Double-buffered on-chip SRAM with AXI DMA engine
- 3D-stacked memory architecture with TSV interface (in progress)
- CXL.mem multi-tile fabric (planned)
- Quasar Computation Graph scheduler over CXL (novel, planned)

## Status

| Phase | Component | Status |
|-------|-----------|--------|
| 0 | CSR file, custom ISA decoder | Done   |
| 1 | MAC unit | Done   |
| 2 | Processing Element | Done   |
| 3 | Systolic Array | Done   |
| 4 | Scratchpad + DMA | Done   |
| 5 | Tile controller | Done   |
| 6 | TSV memory interface | Done   |

## Built With

- [Chisel](https://www.chisel-lang.org/) — hardware description
- [riscv-mini](https://github.com/ucb-bar/riscv-mini) — host CPU base
- [Axiom](https://github.com/SycamoreQ/axiom) — target inference engine (Rust)

## Running Tests

\`\`\`bash
sbt test          # all tests including riscv-mini baseline
sbt "testOnly npu.*"  # Quasar NPU tests only
\`\`\`