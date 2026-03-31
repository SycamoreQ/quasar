package ALU

import chisel3._
import chisel3.util._
import decoder.{FU_t, INT_Op, BR_Op}
import ROB.{FlagsUOp, Flags, R_UOp, TagConst}
import fetch.BranchProv
import branch_pred.SqN


// ExecUOp — uop presented to an execution unit after operand read
// Contains the R_UOp plus resolved operand values from the register file
class ExecUOp extends Bundle {
  val uop    = new R_UOp(4)
  val srcA = UInt(32.W)    // resolved operand A (from reg file or immediate)
  val srcB = UInt(32.W)    // resolved operand B (from reg file or immediate)
  val pc = UInt(32.W)    // PC of this instruction (needed by branch unit)
}

// ExecResult — result produced by an execution unit
// Broadcast on the common data bus (CDB)
class ExecResult extends Bundle {
  val valid   = Bool()
  val tagDst = UInt(TagConst.TAG_SIZE.W)  // physical destination tag
  val result= UInt(32.W)                  // computed result value
  val sqN  = new SqN
}

// IntALU
//
// Single-cycle integer ALU. Handles FU_INT operations.
// Purely combinational — result is available the same cycle as input.
//
// Supported operations (INT_Op):
//   ADD, SUB, SLL, SRL, SRA, AND, OR, XOR, SLT, SLTU, LUI, SYS
//
// Operand selection:
//   srcA — always from register file (or 0 for LUI)
//   srcB — from register file when immB=false, from imm when immB=true


class IntALU extends Module {

  val io = IO(new Bundle {
    val in        = Input(new ExecUOp)
    val out       = Output(new ExecResult)
    val flagsOut  = Output(new FlagsUOp)
  })

  val uop    = io.in.uop
  val srcA   = io.in.srcA
  val srcB   = Mux(uop.immB, uop.imm, io.in.srcB)
  val shamt  = srcB(4, 0)   // shift amount — lower 5 bits only

  // Default outputs
  io.out.valid  := uop.valid && uop.fu === FU_t.FU_INT.asUInt
  io.out.tagDst := uop.tagDst
  io.out.sqN    := uop.sqN
  io.out.result := 0.U

  io.flagsOut.valid       := uop.valid && uop.fu === FU_t.FU_INT.asUInt
  io.flagsOut.sqN         := uop.sqN
  io.flagsOut.tagDst      := uop.tagDst
  io.flagsOut.flags       := Flags.FLAGS_NONE
  io.flagsOut.doNotCommit := false.B

  // Operation decode
  switch(uop.opcode) {
    is(INT_Op.INT_ADD.asUInt)  { io.out.result := srcA + srcB }
    is(INT_Op.INT_SUB.asUInt)  { io.out.result := srcA - srcB }
    is(INT_Op.INT_SLL.asUInt)  { io.out.result := srcA << shamt }
    is(INT_Op.INT_SRL.asUInt)  { io.out.result := srcA >> shamt }
    is(INT_Op.INT_SRA.asUInt)  { io.out.result := (srcA.asSInt >> shamt).asUInt }
    is(INT_Op.INT_AND.asUInt)  { io.out.result := srcA & srcB }
    is(INT_Op.INT_OR.asUInt)   { io.out.result := srcA | srcB }
    is(INT_Op.INT_XOR.asUInt)  { io.out.result := srcA ^ srcB }
    is(INT_Op.INT_SLT.asUInt)  {
      io.out.result := (srcA.asSInt < srcB.asSInt).asUInt
    }
    is(INT_Op.INT_SLTU.asUInt) {
      io.out.result := (srcA < srcB).asUInt
    }
    is(INT_Op.INT_LUI.asUInt)  {
      // LUI — result is the upper immediate, srcA ignored
      io.out.result := uop.imm
    }
    is(INT_Op.INT_SYS.asUInt)  {
      // FENCE.I — result is irrelevant, ROB handles serialization
      // Flags set to FLAGS_FENCE to trigger pipeline drain
      io.out.result           := 0.U
      io.flagsOut.flags       := Flags.FLAGS_FENCE
    }
  }
}


// BranchUnit
//
// Handles FU_BRANCH operations — JAL, JALR variants, conditional branches,
// and AUIPC. Single cycle.
//
// For conditional branches:
//   Evaluates the branch condition from srcA and srcB.
//   Compares the resolved target against the predicted target from the uop.
//   If they differ produces a BranchProv misprediction signal.
//
// For JAL/JALR:
//   The predicted target is in uop.imm (set by the decoder).
//   The resolved target is computed here.
//   rd receives PC+4 (or PC+2 for compressed).
//
// For AUIPC:
//   result = PC + imm (upper immediate)


class BranchUnit extends Module {

  val io = IO(new Bundle {
    val in         = Input(new ExecUOp)
    val out        = Output(new ExecResult)
    val flagsOut   = Output(new FlagsUOp)
    val branchProv = Output(new BranchProv)   // misprediction signal
  })

  val uop  = io.in.uop
  val srcA = io.in.srcA
  val srcB = io.in.srcB
  val pc   = io.in.pc

  // Default outputs — overridden per opcode below
  io.out.valid  := uop.valid && uop.fu === FU_t.FU_BRANCH.asUInt
  io.out.tagDst := uop.tagDst
  io.out.sqN    := uop.sqN
  io.out.result := 0.U

  io.flagsOut.valid       := uop.valid && uop.fu === FU_t.FU_BRANCH.asUInt
  io.flagsOut.sqN         := uop.sqN
  io.flagsOut.tagDst      := uop.tagDst
  io.flagsOut.flags       := Flags.FLAGS_BRANCH
  io.flagsOut.doNotCommit := false.B

  // Default branch provenance — not taken, no misprediction
  io.branchProv.taken     := false.B
  io.branchProv.sqN       := uop.sqN
  io.branchProv.fetchid  := uop.fetchID
  io.branchProv.fetchoffs := uop.fetchOffs
  io.branchProv.flush     := false.B
  io.branchProv.dstPC     := 0.U
  io.branchProv.dst       := 0.U
  io.branchProv.histAct   := 0.U.asTypeOf(io.branchProv.histAct)
  io.branchProv.retAct    := 0.U.asTypeOf(io.branchProv.retAct)
  io.branchProv.tgtspec   := 0.U.asTypeOf(io.branchProv.tgtspec)
  io.branchProv.isSCFail  := false.B

  // PC increment — 2 for compressed, 4 for regular
  val pcIncr    = Mux(uop.compressed, 2.U, 4.U)
  val pcPlus    = pc + pcIncr
  val pcPlusRet = Cat(pcPlus(31, 1), 0.U(1.W))  // aligned return address

  // Predicted target from decoder (stored in uop.imm for JAL/JALR)
  val predTarget = uop.imm(31, 1)

  switch(uop.opcode) {

    is(BR_Op.BR_AUIPC.asUInt) {
      // PC + upper immediate
      io.out.result := pc + uop.imm
      io.flagsOut.flags := Flags.FLAGS_NONE
    }

    is(BR_Op.BR_JAL.asUInt) {
      // Unconditional jump — result is return address (PC+4 or PC+2)
      // Target = PC + imm (sign-extended J-type immediate)
      val target = pc + uop.imm
      io.out.result := pcPlusRet

      // Check if predicted target matches resolved target
      val mispred = predTarget =/= target(31, 1)
      when(mispred) {
        io.branchProv.taken  := true.B
        io.branchProv.flush  := true.B
        io.branchProv.dstPC  := target
      }
      io.flagsOut.flags := Flags.FLAGS_BRANCH
    }

    is(BR_Op.BR_V_JALR, BR_Op.BR_V_RET, BR_Op.BR_V_JR) {
      // Jump register — target = (srcA + imm12) with bit 0 cleared
      val target = Cat((srcA + uop.imm12.asSInt.asUInt)(31, 1), 0.U(1.W))
      io.out.result := pcPlusRet

      val mispred = predTarget =/= target(31, 1)
      when(mispred) {
        io.branchProv.taken := true.B
        io.branchProv.flush := true.B
        io.branchProv.dstPC := target
      }
      io.flagsOut.flags := Flags.FLAGS_BRANCH
    }

    is(BR_Op.BR_BEQ.asUInt) {
      val taken  = srcA === srcB
      val target = Mux(taken, pc + uop.imm, pcPlus)
      handleConditional(taken, target)
    }

    is(BR_Op.BR_BNE.asUInt) {
      val taken  = srcA =/= srcB
      val target = Mux(taken, pc + uop.imm, pcPlus)
      handleConditional(taken, target)
    }

    is(BR_Op.BR_BLT.asUInt) {
      val taken  = srcA.asSInt < srcB.asSInt
      val target = Mux(taken, pc + uop.imm, pcPlus)
      handleConditional(taken, target)
    }

    is(BR_Op.BR_BGE.asUInt) {
      val taken  = srcA.asSInt >= srcB.asSInt
      val target = Mux(taken, pc + uop.imm, pcPlus)
      handleConditional(taken, target)
    }

    is(BR_Op.BR_BLTU.asUInt) {
      val taken  = srcA < srcB
      val target = Mux(taken, pc + uop.imm, pcPlus)
      handleConditional(taken, target)
    }

    is(BR_Op.BR_BGEU.asUInt) {
      val taken  = srcA >= srcB
      val target = Mux(taken, pc + uop.imm, pcPlus)
      handleConditional(taken, target)
    }
  }

  // Helper — shared logic for all conditional branches
  // Sets flags and misprediction signal based on resolved vs predicted outcome
  def handleConditional(taken: Bool, target: UInt): Unit = {
    io.flagsOut.flags := Mux(taken,
      Flags.FLAGS_PRED_TAKEN,
      Flags.FLAGS_PRED_NTAKEN
    )
    io.out.result := 0.U  // conditional branches write no register

    // Misprediction — predicted taken but actually not taken, or vice versa
    // Also misprediction if taken but target address differs
    val predTaken    = uop.imm(0)  // decoder stores pred taken in bit 0 of imm
    val predMismatch = (predTaken =/= taken) ||
                       (taken && predTarget =/= target(31, 1))
    when(predMismatch) {
      io.branchProv.taken  := true.B
      io.branchProv.flush  := true.B
      io.branchProv.dstPC  := target
    }
  }
}
