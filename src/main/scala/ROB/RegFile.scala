package ROB

import chisel3._
import chisel3.util._

// RegFile — Physical Register File
// Stores 128 physical registers of 32 bits each. Physical register 0 is
// hardwired to zero — writes to it are ignored and reads always return 0.
//
// Port configuration matches decode width:
//   8 read ports  — 2 per decode slot (rs1 + rs2 for 4 instructions)
//   4 write ports — 1 per decode slot (rd for 4 instructions)
// Write-before-read within the same cycle — if a write port and a read port
// target the same physical register in the same cycle the read returns the
// newly written value. This is required for correct forwarding when back-to-
// back dependent instructions commit in the same cycle.

// The register file is indexed by physical tag lower bits — the MSB special
// flag is stripped before indexing. Callers must check the special flag
// before issuing reads or writes.
//
// Parameters:
//   NUM_PHYS    — number of physical registers (128)
//   DATA_W      — register width in bits (32 for RV32I)
//   NUM_RD      — number of read ports (8)
//   NUM_WR      — number of write ports (4)
//   RF_TAG_SIZE — physical register index width (7 bits for 128 registers)

class RegFile(
  NUM_PHYS    : Int = 128,
  DATA_W      : Int = 32,
  NUM_RD      : Int = 8,
  NUM_WR      : Int = 4,
  RF_TAG_SIZE : Int = 7    // log2(128)
) extends Module {

  val io = IO(new Bundle {
    // Read ports
    val rdAddr = Input(Vec(NUM_RD, UInt(RF_TAG_SIZE.W)))
    val rdData = Output(Vec(NUM_RD, UInt(DATA_W.W)))

    // Write ports — sequential, written on clock edge
    val wrEn   = Input(Vec(NUM_WR, Bool()))
    val wrAddr = Input(Vec(NUM_WR, UInt(RF_TAG_SIZE.W)))
    val wrData = Input(Vec(NUM_WR, UInt(DATA_W.W)))
  })

  // Reg Vec
  // Physical register 0 is hardwired to zero (x0 in RISC-V)
  val regs = RegInit(VecInit(Seq.fill(NUM_PHYS)(0.U(DATA_W.W))))

  // Priority: higher index write port wins on same-address conflict
  // This matches the rename table commit order — later commits override earlier
  for (i <- 0 until NUM_WR) {
    when(io.wrEn(i) && io.wrAddr(i) =/= 0.U) {
      regs(io.wrAddr(i)) := io.wrData(i)
    }
  }

  // For each read port check if any write port this cycle targets the same
  // address — if so forward the written value directly without waiting for
  // the register update to propagate on the next clock edge.
  for (i <- 0 until NUM_RD) {
    // Default — read from register array
    io.rdData(i) := regs(io.rdAddr(i))

    // Physical register 0 always reads as zero
    when(io.rdAddr(i) === 0.U) {
      io.rdData(i) := 0.U
    }

    // Write forwarding — check all write ports for same-cycle conflict
    // Higher index write port takes priority (last write wins)
    for (j <- 0 until NUM_WR) {
      when(io.wrEn(j) &&
           io.wrAddr(j) === io.rdAddr(i) &&
           io.wrAddr(j) =/= 0.U) {
        io.rdData(i) := io.wrData(j)
      }
    }
  }
}
