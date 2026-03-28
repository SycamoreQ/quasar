package ROB

import chisel3._
import chisel3.util._
import ROB.TagBuffer
import ROB.TagConst.TAG_ZERO
import branch_pred.SqN
import decoder.FU_t.FU_RN
import decoder.{D_UOp, LSU_Op}
import fetch.{BranchProv, FetchID}

/*
Rename sits between the decoder and the issue queues.
It takes four D_UOp bundles per cycle from the decoder and produces four R_UOp bundles for the issue queues and ROB.
 Its job is threefold — assign sequence numbers, allocate physical registers for destinations,
 and look up physical tags for source operands.
 */

class Rename {
  val io = IO(new Bundle{
    val IN_lookupIDs = Input(VecInit(new FetchID))
    val IN_lookupValid = Input(Bool())
    val IN_mispr = Input(Bool())
    val IN_mispredFlush = Input(Bool())
    val IN_branch = Input(new BranchProv)
    val IN_uop = Input(VecInit(new D_UOp))
    val IN_issueValid = Input(Bool())
    val IN_issueTags = Input(UInt(TagConst.TAG_SIZE.W))
    val IN_issueTagsValid = Input(Bool())
    val IN_issueIDs = Input(VecInit(new FetchID))
    val IN_issueAvail = Input(Bool())
    val IN_commitValid = Input(Bool())
    val IN_commitIDs = Input(VecInit(new FetchID))
    val IN_commitTags = Input(UInt(TagConst.TAG_SIZE.W))
    val IN_comUop = Input(new CommitUOp)
    val IN_portStall = Input(Bool())
    val IN_wbValid = Input(Bool())
    val IN_wbTag = Input(UInt(TagConst.TAG_SIZE.W))
    val IN_flagsUop = Input(new FlagsUOp)

    val OUT_nextSqN = Output(new SqN)
    val Out_nextLoadSqN = Output(new SqN)
    val OUT_nextStoreSqN = Output(new SqN)

    val OUT_stall = Output(Bool())
    val OUT_uop = Output(new R_UOp(4))
  })

  val counterSqN      = RegInit(0.U(32.W))
  val counterLoadSqN  = RegInit(0.U(32.W))
  val counterStoreSqN = RegInit(0.U(32.W))
  val failSc          = RegInit(false.B)

  val tag_buf = Module(new TagBuffer())
  val rat= Module(new RenameTable())

  io.OUT_stall := io.IN_portStall.orR || io.IN_mispredFlush

  for (i <- 0 until 4) {
    rat.io.IN_lookupIDs(2*i)   := io.IN_uop(i).rs1
    rat.io.IN_lookupIDs(2*i+1) := io.IN_uop(i).rs2
  }

  io.IN_issueValid := rat.io.IN_issueValid
  io.IN_issueIDs := rat.io.IN_issueIDs
  io.IN_issueAvail := rat.io.IN_issueAvail
  io.IN_issueTags := rat.io.IN_issueTags

  val sqNs      = Wire(Vec(4, UInt(32.W)))
  val loadSqNs  = Wire(Vec(4, UInt(32.W)))
  val storeSqNs = Wire(Vec(4, UInt(32.W)))

  var currentSqN      = counterSqN
  var currentLoadSqN  = counterLoadSqN
  var currentStoreSqN = counterStoreSqN


  for (i <- 0 until 4) {
    val isVld = io.IN_uop(i).valid && !io.OUT_stall
    sqNs(i)      := currentSqN
    loadSqNs(i)  := currentLoadSqN
    storeSqNs(i) := currentStoreSqN

    // Increment if valid and is a Load/Store
    currentSqN      = currentSqN + isVld.asUInt
    currentLoadSqN  = currentLoadSqN + (isVld && io.IN_uop(i).isLoad).asUInt
    currentStoreSqN = currentStoreSqN + (isVld && io.IN_uop(i).isStore).asUInt
  }

  val newTags = Wire(Vec(4, UInt(TagConst.TAG_SIZE.W)))

  for (i <- 0 until 4) {
    val isSc = io.IN_uop(i).opcode === LSU_Op.LSU_SC_W.asUInt
    val scFailed = isSc && failSc //

    newTags(i) := MuxCase(TAG_ZERO, Seq(
      (io.IN_uop(i).fu === FU_RN) -> io.IN_uop(i).imm, // Immediate rename
      scFailed -> TagConst.TAG_SC_FAIL,       // SC failure
      (io.IN_uop(i).rd =/= 0.U) -> tag_buf.io.tags(i) // Normal destination
    ))
  }






}