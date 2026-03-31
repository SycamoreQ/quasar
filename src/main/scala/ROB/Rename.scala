package ROB

import chisel3._
import chisel3.util._
import fetch.BranchProv
import branch_pred.SqN
import decoder.{D_UOp, FU_t, LSU_Op}

class Rename(
  WIDTH           : Int = 4,
  NUM_PORTS_TOTAL : Int = 4,
  TAG_SIZE        : Int = 8
) extends Module {

  val io = IO(new Bundle {
    val frontEn         = Input(Bool())
    val IN_stalls       = Input(Vec(NUM_PORTS_TOTAL, UInt(WIDTH.W)))
    val OUT_stall       = Output(Bool())

    val IN_uop          = Input(Vec(WIDTH, new D_UOp))
    val IN_comUOp       = Input(Vec(WIDTH, new CommitUOp))
    val IN_flagsUOps    = Input(Vec(WIDTH, new FlagsUOp))
    val IN_branch       = Input(new BranchProv)
    val IN_mispredFlush = Input(Bool())

    val OUT_uop          = Output(Vec(WIDTH, new R_UOp(NUM_PORTS_TOTAL)))
    val OUT_nextSqN      = Output(new SqN)
    val OUT_nextLoadSqN  = Output(new SqN)
    val OUT_nextStoreSqN = Output(new SqN)
  })

  val counterSqN      = RegInit(0.U(32.W))
  val counterLoadSqN  = RegInit(0.U(32.W))
  val counterStoreSqN = RegInit((-1).S(32.W).asUInt)  // starts at -1 matching ROB
  val failSc          = RegInit(false.B)


  val rat     = Module(new RenameTable(
    NUM_LOOKUP = WIDTH * 2,
    NUM_ISSUE  = WIDTH,
    NUM_COMMIT = WIDTH,
    NUM_WB     = WIDTH,
    TAG_SIZE   = TAG_SIZE
  ))
  val tagBuf  = Module(new TagBuffer(
    NUM_ISSUE  = WIDTH,
    NUM_COMMIT = WIDTH,
    TAG_SIZE   = TAG_SIZE
  ))

  // Stall if any issue queue port is stalled, mispred flush is active,
  // or TagBuffer does not have enough free tags for pending instructions
  val portStall = io.IN_stalls.map(_.orR).reduce(_ || _)

  // A tag is needed when rd != 0 and fu is not RN, TRAP, or failed SC
  val isSc      = Wire(Vec(WIDTH, Bool()))
  val scFailed  = Wire(Vec(WIDTH, Bool()))
  val tagNeeded = Wire(Vec(WIDTH, Bool()))
  for (i <- 0 until WIDTH) {
    isSc(i)      := io.IN_uop(i).fu === FU_t.FU_AGU.asUInt &&
                    io.IN_uop(i).opcode === LSU_Op.LSU_SC_W.asUInt
    scFailed(i)  := isSc(i) && failSc
    tagNeeded(i) := io.IN_uop(i).rd =/= 0.U &&
                    io.IN_uop(i).fu =/= FU_t.FU_RN.asUInt &&
                    io.IN_uop(i).fu =/= FU_t.FU_TRAP.asUInt &&
                    !scFailed(i)
  }

  // Stall if any needed tag is not available from TagBuffer
  val tagStall = (0 until WIDTH).map(i =>
    io.IN_uop(i).valid && tagNeeded(i) && !tagBuf.io.OUT_issueTagsValid(i)
  ).reduce(_ || _)

  io.OUT_stall := portStall || io.IN_mispredFlush || tagStall

  val cycleValid = !io.IN_branch.taken && io.frontEn && !io.OUT_stall

  // Two lookup slots per instruction — rs1 at 2*i, rs2 at 2*i+1
  for (i <- 0 until WIDTH) {
    rat.io.IN_lookupIDs(2 * i)     := io.IN_uop(i).rs1
    rat.io.IN_lookupIDs(2 * i + 1) := io.IN_uop(i).rs2
  }

  // RAT issue ports — driven from current dispatch group
  for (i <- 0 until WIDTH) {
    rat.io.IN_issueValid(i) := cycleValid && io.IN_uop(i).valid
    rat.io.IN_issueIDs(i)   := io.IN_uop(i).rd
    rat.io.IN_issueTags(i)  := newTags(i)   // defined below
    rat.io.IN_issueAvail(i) := io.IN_uop(i).fu === FU_t.FU_RN.asUInt || scFailed(i)
  }

  // RAT misprediction control
  rat.io.IN_mispred      := io.IN_branch.taken
  rat.io.IN_mispredFlush := io.IN_mispredFlush

  for (i <- 0 until WIDTH) {
    tagBuf.io.IN_issueValid(i) := cycleValid &&
                                   io.IN_uop(i).valid &&
                                   tagNeeded(i)
  }
  tagBuf.io.IN_mispr        := io.IN_branch.taken
  tagBuf.io.IN_mispredFlush := io.IN_mispredFlush

  val sqNs      = Wire(Vec(WIDTH + 1, UInt(32.W)))
  val loadSqNs  = Wire(Vec(WIDTH + 1, UInt(32.W)))
  val storeSqNs = Wire(Vec(WIDTH + 1, UInt(32.W)))

  sqNs(0)      := counterSqN
  loadSqNs(0)  := counterLoadSqN
  storeSqNs(0) := counterStoreSqN

  for (i <- 0 until WIDTH) {
    val isVld = cycleValid && io.IN_uop(i).valid && !scFailed(i)

    val isLd = io.IN_uop(i).fu === FU_t.FU_AGU.asUInt &&
               io.IN_uop(i).opcode < LSU_Op.LSU_SC_W.asUInt ||
               io.IN_uop(i).fu === FU_t.FU_ATOMIC.asUInt

    val isSt = io.IN_uop(i).fu === FU_t.FU_AGU.asUInt &&
               io.IN_uop(i).opcode >= LSU_Op.LSU_SC_W.asUInt ||
               io.IN_uop(i).fu === FU_t.FU_ATOMIC.asUInt

    sqNs(i + 1)      := sqNs(i)      + isVld.asUInt
    loadSqNs(i + 1)  := loadSqNs(i)  + (isVld && isLd).asUInt
    storeSqNs(i + 1) := storeSqNs(i) + (isVld && isSt).asUInt
  }

  io.OUT_nextSqN.value      := sqNs(WIDTH)
  io.OUT_nextLoadSqN.value  := loadSqNs(WIDTH)
  io.OUT_nextStoreSqN.value := storeSqNs(WIDTH)

  // newTags is referenced in RAT wiring above — declare as Wire first
  val newTags = Wire(Vec(WIDTH, UInt(TAG_SIZE.W)))

  for (i <- 0 until WIDTH) {
    newTags(i) := MuxCase(TagConst.TAG_ZERO, Seq(
      // FU_RN — eliminated at rename, encode immediate as special tag
      (io.IN_uop(i).fu === FU_t.FU_RN.asUInt) ->
        Cat(1.U(1.W), io.IN_uop(i).imm(TAG_SIZE - 2, 0)),
      // Failed SC.W — encode failure result as special tag with value 1
      scFailed(i) ->
        Cat(1.U(1.W), 1.U((TAG_SIZE - 1).W)),
      // Normal destination register — allocate physical tag from TagBuffer
      (io.IN_uop(i).rd =/= 0.U &&
       io.IN_uop(i).fu =/= FU_t.FU_TRAP.asUInt) ->
        Cat(0.U(1.W), tagBuf.io.OUT_issueTags(i))
    ))
  }

  // Driven from ROB CommitUOp — updates committed maps and frees old tags

  // Detect newest commit for each rd — needed by TagBuffer to avoid
  // double-freeing when the same rd is committed multiple times
  val isNewestCommit = Wire(Vec(WIDTH, Bool()))
  for (i <- 0 until WIDTH) {
    isNewestCommit(i) := io.IN_comUOp(i).valid && io.IN_comUOp(i).rd =/= 0.U
    for (j <- i + 1 until WIDTH) {
      when(io.IN_comUOp(j).valid &&
           io.IN_comUOp(j).rd === io.IN_comUOp(i).rd) {
        isNewestCommit(i) := false.B
      }
    }
  }

  for (i <- 0 until WIDTH) {
    // RAT commit ports
    rat.io.IN_commitValid(i) := io.IN_comUOp(i).valid &&
                                  io.IN_comUOp(i).rd =/= 0.U
    rat.io.IN_commitIDs(i)   := io.IN_comUOp(i).rd
    rat.io.IN_commitTags(i)  := io.IN_comUOp(i).tagDst

    // TagBuffer commit ports
    tagBuf.io.IN_commitValid(i)          := io.IN_comUOp(i).valid
    tagBuf.io.IN_commitNewest(i)         := isNewestCommit(i)
    tagBuf.io.IN_commitTagDst(i)         := io.IN_comUOp(i).tagDst
    tagBuf.io.IN_RAT_commitPrevTags(i)   := rat.io.OUT_commitPrevTags(i)
  }

  for (i <- 0 until WIDTH) {
    rat.io.IN_wbValid(i) := io.IN_flagsUOps(i).valid &&
                             !io.IN_flagsUOps(i).tagDst(TAG_SIZE - 1)
    rat.io.IN_wbTag(i)   := io.IN_flagsUOps(i).tagDst
  }

  // Default all outputs to invalid
  for (i <- 0 until WIDTH) {
    io.OUT_uop(i) := 0.U.asTypeOf(new R_UOp(NUM_PORTS_TOTAL))
  }

  when(cycleValid) {
    for (i <- 0 until WIDTH) {
      when(io.IN_uop(i).valid) {
        io.OUT_uop(i).valid      := true.B
        io.OUT_uop(i).validIQ    := ~0.U(NUM_PORTS_TOTAL.W)
        io.OUT_uop(i).sqN.value  := sqNs(i)
        io.OUT_uop(i).loadSqN.value  := loadSqNs(i)
        io.OUT_uop(i).storeSqN.value := storeSqNs(i + 1) // store SqN pre-increments

        io.OUT_uop(i).tagDst  := newTags(i)
        io.OUT_uop(i).tagA    := rat.io.OUT_lookupSpecTag(2 * i)
        io.OUT_uop(i).availA  := rat.io.OUT_lookupAvail(2 * i)
        io.OUT_uop(i).tagB    := rat.io.OUT_lookupSpecTag(2 * i + 1)
        io.OUT_uop(i).availB  := rat.io.OUT_lookupAvail(2 * i + 1)

        // Third source for atomics — the memory operand tag is the same
        // as tagDst since the atomic reads and writes the same location
        io.OUT_uop(i).tagC   := TagConst.TAG_ZERO
        io.OUT_uop(i).availC := true.B
        when(io.IN_uop(i).fu === FU_t.FU_ATOMIC.asUInt) {
          io.OUT_uop(i).tagC   := newTags(i)
          io.OUT_uop(i).availC := false.B
        }

        // Pass-through fields from D_UOp
        io.OUT_uop(i).rd         := io.IN_uop(i).rd
        io.OUT_uop(i).fu         := io.IN_uop(i).fu
        io.OUT_uop(i).opcode     := io.IN_uop(i).opcode
        io.OUT_uop(i).imm        := io.IN_uop(i).imm
        io.OUT_uop(i).imm12      := io.IN_uop(i).imm12
        io.OUT_uop(i).immB       := io.IN_uop(i).immB
        io.OUT_uop(i).fetchID    := io.IN_uop(i).fetchID
        io.OUT_uop(i).fetchOffs  := io.IN_uop(i).fetchOffs
        io.OUT_uop(i).compressed := io.IN_uop(i).compressed

        // Trap instructions encode the trap cause in rd for the ROB
        when(io.IN_uop(i).fu === FU_t.FU_TRAP.asUInt) {
          io.OUT_uop(i).rd := io.IN_uop(i).opcode(4, 0)
        }

        // Failed SC.W — treat as FU_RN, no execution needed
        when(scFailed(i)) {
          io.OUT_uop(i).fu := FU_t.FU_RN.asUInt
        }
      }
    }
  }

  when(io.IN_branch.taken) {
    // Reset sequence number counters to branch resolution point
    counterSqN      := io.IN_branch.sqN.value + 1.U
    counterLoadSqN  := io.IN_branch.loadSqN
    counterStoreSqN := io.IN_branch.storeSqN

    failSc := io.IN_branch.isSCFail

    // Squash any output uops past the mispredicting instruction
    for (i <- 0 until WIDTH) {
      when((io.OUT_uop(i).sqN.value.asSInt -
            io.IN_branch.sqN.value.asSInt) > 0.S) {
        io.OUT_uop(i).valid  := false.B
        io.OUT_uop(i).validIQ := 0.U
      }
    }
  }.otherwise {
    // Normal advance — update counters
    counterSqN      := sqNs(WIDTH)
    counterLoadSqN  := loadSqNs(WIDTH)
    counterStoreSqN := storeSqNs(WIDTH)
  }

  // When stalled — keep validIQ bits set for stalled ports
  // Update tag availability from any writebacks that arrive while stalled
  when(portStall) {
    for (i <- 0 until WIDTH) {
      when(io.OUT_uop(i).validIQ.orR) {
        for (j <- 0 until WIDTH) {
          when(io.IN_flagsUOps(j).valid &&
               !io.IN_flagsUOps(j).tagDst(TAG_SIZE - 1)) {
            when(io.OUT_uop(i).tagA === io.IN_flagsUOps(j).tagDst) {
              io.OUT_uop(i).availA := true.B
            }
            when(io.OUT_uop(i).tagB === io.IN_flagsUOps(j).tagDst) {
              io.OUT_uop(i).availB := true.B
            }
            when(io.OUT_uop(i).tagC === io.IN_flagsUOps(j).tagDst) {
              io.OUT_uop(i).availC := true.B
            }
          }
        }
        // Clear validIQ bits for ports that are no longer stalled
        for (j <- 0 until NUM_PORTS_TOTAL) {
          when(!io.IN_stalls(j).orR) {
            io.OUT_uop(i).validIQ := io.OUT_uop(i).validIQ & ~(1.U << j.U).asUInt
          }
        }
      }
    }
  }
}
