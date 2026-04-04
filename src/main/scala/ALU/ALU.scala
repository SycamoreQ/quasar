package ALU

import chisel3._
import chisel3.util._
import ROB.{FlagsUOp, TagConst}
import fetch.BranchProv

// ALU — Top-level integer execution unit
//
// Dispatches incoming ExecUOp to the correct sub-unit based on fu field,
// collects results, and drives writeback, CDB broadcast, ROB flags,
// and branch misprediction signals.
//
// Sub-units:
//   IntALU    — handles FU_INT (single cycle integer operations)
//   BranchUnit — handles FU_BRANCH (branches, jumps, AUIPC)
//
// MulUnit and DivUnit are instantiated separately and connected at the
// issue queue level since they have variable latency.

class ALU(
  RF_TAG_SIZE : Int = 7,   // physical register index width
  TAG_SIZE    : Int = 8    // full tag width including MSB special flag
) extends Module {

  val io = IO(new Bundle {
    // From issue queue — uop with resolved operands
    val in = Input(new ExecUOp)

    // To register file — writeback
    val wrEn   = Output(Bool())
    val wrAddr = Output(UInt(RF_TAG_SIZE.W))
    val wrData = Output(UInt(32.W))

    // To ROB — completion flag
    val flagsOut = Output(new FlagsUOp)

    // To entire pipeline — branch misprediction
    val branchProv = Output(new BranchProv)

    // To issue queue — CDB broadcast for operand wakeup
    val cdbValid = Output(Bool())
    val cdbTag   = Output(UInt(TAG_SIZE.W))
    val cdbData  = Output(UInt(32.W))
  })

  //Sub-unit instantiation
  val intALU     = Module(new IntALU)
  val branchUnit = Module(new BranchUnit)

  //Wire inputs to both sub-units
  // Both receive the same uop — each gates its output on its own fu check
  intALU.io.in     := io.in
  branchUnit.io.in := io.in

  //Default outputs
  io.wrEn        := false.B
  io.wrAddr      := 0.U
  io.wrData      := 0.U
  io.flagsOut    := 0.U.asTypeOf(new FlagsUOp)
  io.branchProv  := 0.U.asTypeOf(new BranchProv)
  io.cdbValid    := false.B
  io.cdbTag      := TagConst.TAG_ZERO
  io.cdbData     := 0.U

  // Only one sub-unit produces a valid result per cycle since fu is one-hot.
  // Priority: IntALU first, BranchUnit second.

  when(intALU.io.out.valid) {
    // IntALU result — write back to register file and broadcast on CDB
    val tagIsReal = !intALU.io.out.tagDst(TAG_SIZE - 1)  // MSB=0 means real physical reg

    io.wrEn    := tagIsReal
    io.wrAddr  := intALU.io.out.tagDst(RF_TAG_SIZE - 1, 0)
    io.wrData  := intALU.io.out.result

    io.cdbValid := tagIsReal
    io.cdbTag   := intALU.io.out.tagDst
    io.cdbData  := intALU.io.out.result

    io.flagsOut := intALU.io.flagsOut

  }.elsewhen(branchUnit.io.out.valid) {
    // BranchUnit result — write return address for JAL/JALR, no write for Bxx
    val tagIsReal = !branchUnit.io.out.tagDst(TAG_SIZE - 1)

    io.wrEn    := tagIsReal
    io.wrAddr  := branchUnit.io.out.tagDst(RF_TAG_SIZE - 1, 0)
    io.wrData  := branchUnit.io.out.result

    io.cdbValid := tagIsReal
    io.cdbTag   := branchUnit.io.out.tagDst
    io.cdbData  := branchUnit.io.out.result

    io.flagsOut    := branchUnit.io.flagsOut
    io.branchProv  := branchUnit.io.branchProv
  }
}
