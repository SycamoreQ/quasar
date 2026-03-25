package ROB

import ROB.Flags.FLAGS_NX
import chisel3._
import chisel3.util._
import fetch.{BPUpdate, BranchProv, FetchID, FetchOff, iFetchParams}
import branch_pred.SqN
import decoder.{FU_t, INT_Op, LSU_Op}

/*
The ROB is purely a tracking and commit structure
 */

class ROB(
           ID_LEN      : Int = 5,
           WIDTH       : Int = 4,       // commit width = DEC_WIDTH
           WIDTH_RN    : Int = 4,       // rename/dispatch width
           NUM_FLAG_UOPS : Int = 4,     // number of execution ports
           NUM_AGUS    : Int = 2        // number of address generation units
         ) extends Module {

  val LENGTH = 1 << ID_LEN // 32 entries

  val io = IO(new Bundle {
    // From Rename — dispatch
    val IN_uop = Input(Vec(WIDTH_RN, new R_UOp(WIDTH)))

    // From execution units — completion flags
    val IN_flagUOps = Input(Vec(NUM_FLAG_UOPS, new FlagsUOp))

    // From execute — branch misprediction
    val IN_branch = Input(new BranchProv)

    // From LSU — commit limits
    val IN_stComLimit = Input(Vec(NUM_AGUS, new ComLimit))
    val IN_ldComLimit = Input(new ComLimit)

    // Interrupt from CSR/PLIC
    val IN_interruptPending = Input(Bool())

    // To Rename — flow control
    val OUT_maxSqN = Output(new SqN)
    val OUT_curSqN = Output(new SqN)

    // To LSU — committed load/store tracking
    val OUT_lastLoadSqN = Output(new SqN)
    val OUT_lastStoreSqN = Output(new SqN)

    // To register file and rename table — commit
    val OUT_comUOp = Output(Vec(WIDTH, new CommitUOp))

    // To trap handler — exceptions
    val OUT_trapUOp = Output(new Trap_UOp)

    // To branch predictor — branch outcome at commit
    val OUT_bpUpdate = Output(new BPUpdate)

    // To fetch — current committed fetch ID
    val OUT_curFetchID = Output(new FetchID)

    // Pipeline flush signal
    val OUT_mispredFlush = Output(Bool())

    // Performance counters
    val OUT_perfcInfo = Output(new ROB_PERFC_Info(WIDTH))
  })

  //Entry banks — one per decode slot, banked on SqN % WIDTH
  //Using Mem (not SyncReadMem) so reads are combinational during commit
  val entries = Seq.fill(WIDTH)(Mem(LENGTH / WIDTH, new ROBEntry))

  // Flags array — written OoO by execution units, read in-order at commit
  //Must be combinational read — use Reg Vec not SyncReadMem
  val flags = RegInit(VecInit(Seq.fill(LENGTH)(Flags.FLAGS_NX)))

  //Circular buffer pointers
  val baseIndex = RegInit(0.U((ID_LEN + 1).W)) // commit head
  val lastIndex = RegInit(0.U((ID_LEN + 1).W)) // dispatch tail

  // Load/store SqN tracking at commit boundary
  val loadSqN_r = RegInit(0.U((ID_LEN + 1).W))
  val storeSqN_r = RegInit(((1 << (ID_LEN + 1)) - 1).U((ID_LEN + 1).W)) // -1

  // Misprediction replay state
  val misprReplay_r = RegInit(0.U.asTypeOf(new MisprReplay))

  // Hang detection
  val hangCounter = RegInit(0.U(HangConst.HANG_COUNTER_LEN.W))
  val hangDetected = RegInit(false.B)
  val didCommit = RegInit(false.B)

  io.OUT_maxSqN := (baseIndex + (LENGTH - 1).U).asTypeOf(new SqN)
  io.OUT_curSqN := baseIndex.asTypeOf(new SqN)

  val rnUopSorted = WireDefault(VecInit(Seq.fill(WIDTH_RN)(0.U.asTypeOf(new R_UOp(4)))))


  // sorting the uops to be in order and put them in a separate rename sorted buff
  for (i <- 0 until WIDTH_RN) {
    for (j <- 0 until WIDTH_RN) {
      when(io.IN_uop(j).valid && io.IN_uop(j).sqN.value(log2Ceil(WIDTH_RN) - 1, 0) === i.U) {
        rnUopSorted(i) := io.IN_uop(j)
      }
    }

    when(rnUopSorted(i).valid === true.B && io.IN_branch.taken === false.B) {
      val bankBits = log2Ceil(WIDTH_RN)
      val bankIndex = rnUopSorted(i).sqN.value(bankBits - 1, 0)
      val entryIdx = rnUopSorted(i).sqN.value(ID_LEN - 1, bankBits)

      val entry = WireDefault(0.U.asTypeOf(new ROBEntry))
      entry.tag := rnUopSorted(i).tagDst
      entry.rd := rnUopSorted(i).rd
      entry.fetchID := rnUopSorted(i).fetchID
      entry.fetchOffs := rnUopSorted(i).fetchOffs
      entry.compressed := rnUopSorted(i).compressed
      entry.isLd := (rnUopSorted(i).fu === FU_t.FU_AGU.asUInt &&
        rnUopSorted(i).opcode < LSU_Op.LSU_SC_W.asUInt) ||
        rnUopSorted(i).fu === FU_t.FU_ATOMIC.asUInt
      entry.isSt := (rnUopSorted(i).fu === FU_t.FU_AGU.asUInt &&
        rnUopSorted(i).opcode >= LSU_Op.LSU_SC_W.asUInt) ||
        rnUopSorted(i).fu === FU_t.FU_ATOMIC.asUInt

      when(rnUopSorted(i).valid && !io.IN_branch.taken) {
        entries(bankIndex)(entryIdx) := entry

        // Initialize flags for this slot
        val flatIdx = rnUopSorted(i).sqN.value(ID_LEN - 1, 0)
        when(rnUopSorted(i).fu === FU_t.FU_RN.asUInt) {
          flags(flatIdx) := Flags.FLAGS_NONE
        }.elsewhen(rnUopSorted(i).fu === FU_t.FU_TRAP.asUInt) {
          flags(flatIdx) := Flags.FLAGS_TRAP
        }.otherwise {
          flags(flatIdx) := Flags.FLAGS_NX
        }
      }
    }
    when(io.IN_uop(i).valid && !io.IN_branch.taken) {
      lastIndex := io.IN_uop(i).sqN.value + 1.U
    }
  }

  //iterate on flags so that the OoO instrs are ordered back sequentially using the FLAG_NX SqN
  for (i <- 0 until NUM_FLAG_UOPS) {
    val branchCond = !io.IN_branch.taken ||
      (io.IN_flagUOps(i).sqN.value.asSInt -
        io.IN_branch.sqN.value.asSInt) <= 0.S

    when(io.IN_flagUOps(i).valid &&
      io.IN_flagUOps(i).flags =/= Flags.FLAGS_NX &&
      branchCond &&
      !io.IN_flagUOps(i).doNotCommit) {
      val flatIdx = io.IN_flagUOps(i).sqN.value(ID_LEN - 1, 0)
      flags(flatIdx) := io.IN_flagUOps(i).flags
    }
  }

  // this step accums the OoO instrs into proper sequential order for it to be evaulated properly
  // the way this is done by first accum the incoming entries and flags into a vector , and then sorting the entries as
  // the entires cannot go in burst as the buffer is divided into 4 slots each.
  val deqAddrs = Wire(Vec(WIDTH, UInt(ID_LEN.W)))
  val deqAddrsSorted = Wire(Vec(WIDTH, UInt((ID_LEN - log2Ceil(WIDTH)).W)))
  val deqEntries = Wire(Vec(WIDTH, new ROBEntry))
  val deqFlags = Wire(Vec(WIDTH, Flags()))

  // compute raw commit addresses
  for (i <- 0 until WIDTH) {
    deqAddrs(i) := baseIndex(ID_LEN - 1, 0) + i.U
  }

  // sort by lower bits to produce sequential bank access
  for (i <- 0 until WIDTH) {
    deqAddrsSorted(i) := DontCare
  }
  for (i <- 0 until WIDTH) {
    val lowerBits = deqAddrs(i)(log2Ceil(WIDTH) - 1, 0)
    val upperBits = deqAddrs(i)(ID_LEN - 1, log2Ceil(WIDTH))
    deqAddrsSorted(lowerBits) := upperBits
  }

  // read from each bank using sorted addresses
  //reorder back into commit order using deqAddrs lower bits
  for (i <- 0 until WIDTH) {
    val lowerBits = deqAddrs(i)(log2Ceil(WIDTH) - 1, 0)
    deqEntries(i) := entries(lowerBits)(deqAddrsSorted(lowerBits))
    deqFlags(i) := flags(deqAddrs(i))
  }

  def getSqN(idx: UInt): UInt = {
    val hiBits = baseIndex(ID_LEN)
    Mux(idx >= baseIndex(ID_LEN-1, 0),
      Cat(hiBits, idx),
      Cat(hiBits + 1.U, idx))
  }

  // Attempts to retire up to WIDTH instructions per cycle in order.
  // Uses Scala vars as elaboration-time carry signals across loop iterations
  // to enforce strict in-order commit — once one slot fails all subsequent
  // slots in the same cycle are also blocked.

  // Default output assignments — overridden when instructions retire
  for (i <- 0 until WIDTH) {
    io.OUT_comUOp(i).valid      := false.B
    io.OUT_comUOp(i).rd         := 0.U
    io.OUT_comUOp(i).tagDst     := TagConst.TAG_ZERO
    io.OUT_comUOp(i).sqN        := 0.U.asTypeOf(new SqN)
    io.OUT_comUOp(i).isBranch   := false.B
    io.OUT_comUOp(i).compressed := false.B
  }
  io.OUT_trapUOp.valid    := false.B
  io.OUT_bpUpdate.valid   := false.B
  io.OUT_curFetchID       := 0.U.asTypeOf(new FetchID)
  io.OUT_mispredFlush     := false.B
  io.OUT_perfcInfo.validRetire  := 0.U
  io.OUT_perfcInfo.branchRetire := 0.U
  io.OUT_perfcInfo.stallCause   := StallCause.STALL_FRONTEND
  io.OUT_perfcInfo.stallWeight  := 3.U

  // Load/store SqN prefix sums across commit slots
  // storeSqNs(i+1) and loadSqNs(i+1) give the SqN AFTER slot i commits
  val storeSqNs = Wire(Vec(WIDTH + 1, UInt((ID_LEN + 1).W)))
  val loadSqNs  = Wire(Vec(WIDTH + 1, UInt((ID_LEN + 1).W)))
  storeSqNs(0) := storeSqN_r
  loadSqNs(0)  := loadSqN_r
  for (i <- 0 until WIDTH) {
    storeSqNs(i + 1) := storeSqNs(i) + deqEntries(i).isSt.asUInt
    loadSqNs(i + 1)  := loadSqNs(i)  + deqEntries(i).isLd.asUInt
  }

  //carry state across loop iterations at elaboration time
  // These become part of the generated hardware as chained conditions
  var temp  = false.B   // once true, blocks all subsequent slots this cycle
  var pred  = false.B   // true when a branch/ordering flag was seen earlier
  var cnt   = 0.U(log2Ceil(WIDTH + 1).W)  // count of successful commits

  for (i <- 0 until WIDTH) {


    // Slot is occupied — within dispatched range
    val isRenamed = (i.U.asSInt < (lastIndex - baseIndex).asSInt)

    // Execution unit has written back a result
    val isExecuted = deqFlags(i) =/= Flags.FLAGS_NX

    // No earlier slot in this group had a serializing flag
    val noFlagConflict = !pred || (deqFlags(i) === Flags.FLAGS_NONE)

    // Load buffer allows this load to commit
    val lbAllowsCommit = !io.IN_ldComLimit.valid ||
      (loadSqNs(i).asSInt - io.IN_ldComLimit.sqN.value.asSInt) < 0.S

    // All store queues allow this store to commit
    val sqAllowsCommit = (0 until NUM_AGUS).map { j =>
      !io.IN_stComLimit(j).valid ||
        (storeSqNs(i + 1).asSInt - io.IN_stComLimit(j).sqN.value.asSInt) < 0.S
    }.reduce(_ && _)

    // Hang detection forces commit of head instruction after timeout
    val timeoutCommit = (i == 0).B && hangDetected

    when(!temp && isRenamed &&
      ((isExecuted && noFlagConflict && sqAllowsCommit && lbAllowsCommit) ||
        timeoutCommit)) {

      // Reconstruct full SqN from partial index
      val sqN = getSqN(deqAddrs(i))

      io.OUT_comUOp(i).valid      := true.B
      io.OUT_comUOp(i).rd         := deqEntries(i).rd
      io.OUT_comUOp(i).tagDst     := deqEntries(i).tag
      io.OUT_comUOp(i).sqN        := sqN.asTypeOf(new SqN)
      io.OUT_comUOp(i).isBranch   := deqFlags(i) === Flags.FLAGS_PRED_TAKEN  ||
        deqFlags(i) === Flags.FLAGS_PRED_NTAKEN ||
        deqFlags(i) === Flags.FLAGS_BRANCH
      io.OUT_comUOp(i).compressed := deqEntries(i).compressed

      // Update current fetch ID visible to the trap handler
      io.OUT_curFetchID := deqEntries(i).fetchID

      when(deqFlags(i) === Flags.FLAGS_PRED_TAKEN ||
        deqFlags(i) === Flags.FLAGS_PRED_NTAKEN) {
        io.OUT_bpUpdate.valid       := true.B
        io.OUT_bpUpdate.branchTaken := deqFlags(i) === Flags.FLAGS_PRED_TAKEN
        io.OUT_bpUpdate.fetchID     := deqEntries(i).fetchID
        io.OUT_bpUpdate.fetchoff   := deqEntries(i).fetchOffs
        pred = true.B  // block subsequent slots — branch resolves in-order
      }

      // isException: timeout or flags in the exception range
      val isException = timeoutCommit ||
        (deqFlags(i).asUInt >= Flags.FLAGS_ILLEGAL_INSTR.asUInt &&
          deqFlags(i).asUInt <= Flags.FLAGS_ST_AF.asUInt)

      // sendTrapUOp: any serializing flag including FENCE, XRET, TRAP
      val sendTrapUOp = timeoutCommit ||
        deqFlags(i).asUInt >= Flags.FLAGS_FENCE.asUInt

      when(sendTrapUOp) {
        io.OUT_trapUOp.valid      := true.B
        io.OUT_trapUOp.timeout    := timeoutCommit
        io.OUT_trapUOp.flags      := deqFlags(i)
        io.OUT_trapUOp.tag        := deqEntries(i).tag
        io.OUT_trapUOp.sqN        := sqN.asTypeOf(new SqN)
        io.OUT_trapUOp.loadSqN    := loadSqNs(i).asTypeOf(new SqN)
        io.OUT_trapUOp.storeSqN   := storeSqNs(i + 1).asTypeOf(new SqN)
        io.OUT_trapUOp.rd         := deqEntries(i).rd
        io.OUT_trapUOp.fetchOffs  := deqEntries(i).fetchOffs
        io.OUT_trapUOp.fetchID    := deqEntries(i).fetchID
        io.OUT_trapUOp.compressed := deqEntries(i).compressed

        // On exception redirect result to x0 — prevents incorrect
        // register file update while still allowing rename map rollback
        when(isException) {
          io.OUT_comUOp(i).rd     := 0.U
          io.OUT_comUOp(i).tagDst := TagConst.TAG_ZERO
        }

        temp = true.B  // stop committing after a trap
      }

      when(!isException) {
        when(deqEntries(i).isLd) {
          io.OUT_lastLoadSqN  := loadSqNs(i + 1).asTypeOf(new SqN)
          loadSqN_r           := loadSqNs(i + 1)
        }
        when(deqEntries(i).isSt) {
          io.OUT_lastStoreSqN := storeSqNs(i + 1).asTypeOf(new SqN)
          storeSqN_r          := storeSqNs(i + 1)
        }
      }

      io.OUT_perfcInfo.validRetire  := io.OUT_perfcInfo.validRetire  | (1.U << i)
      io.OUT_perfcInfo.branchRetire := io.OUT_perfcInfo.branchRetire |
        (io.OUT_comUOp(i).isBranch.asUInt << i)
      io.OUT_perfcInfo.stallCause   := StallCause.STALL_NONE
      io.OUT_perfcInfo.stallWeight  := (WIDTH - i - 1).U

      didCommit := true.B
      cnt = cnt + 1.U

    }.otherwise {

      // Slot i could not commit — record why for perf counters
      // Only record the first stalled slot
      when(!temp) {
        io.OUT_perfcInfo.stallWeight := (WIDTH - i).U

        when(!isRenamed || temp) {
          io.OUT_perfcInfo.stallCause := StallCause.STALL_FRONTEND
        }.elsewhen(!isExecuted) {
          when(deqEntries(i).isSt) {
            io.OUT_perfcInfo.stallCause := StallCause.STALL_STORE
          }.elsewhen(deqEntries(i).isLd) {
            io.OUT_perfcInfo.stallCause := StallCause.STALL_LOAD
          }.otherwise {
            io.OUT_perfcInfo.stallCause := StallCause.STALL_BACKEND
          }
        }.elsewhen(!sqAllowsCommit) {
          io.OUT_perfcInfo.stallCause := StallCause.STALL_STORE
        }.elsewhen(!lbAllowsCommit) {
          io.OUT_perfcInfo.stallCause := StallCause.STALL_LOAD
        }.elsewhen(!noFlagConflict) {
          io.OUT_perfcInfo.stallCause := StallCause.STALL_ROB
        }

        // Drive trap UOp with NX flags for debugger visibility on slot 0
        when(i.U === 0.U && isRenamed) {
          io.OUT_trapUOp.valid      := true.B
          io.OUT_trapUOp.timeout    := false.B
          io.OUT_trapUOp.flags      := Flags.FLAGS_NX
          io.OUT_trapUOp.fetchOffs  := deqEntries(i).fetchOffs
          io.OUT_trapUOp.fetchID    := deqEntries(i).fetchID
          io.OUT_trapUOp.compressed := deqEntries(i).compressed
        }
      }

      temp = true.B  // block all subsequent slots
    }
  }

  // Advance commit head by number of successful commits this cycle
  baseIndex := baseIndex + cnt

  // mispr replay logic

  val misprReplay_R = RegInit(new MisprReplay  , (baseIndex -1.U , 0))
  val misprReplay_c = Wire(new MisprReplay)



  //When a branch misprediction is detected the execute unit knows exactly which instruction caused the misprediction
  // it tags the result with that instruction's SqN and sends it back via IN_branch.
  // iterSqN iterates over the sequence of instructions to be replayed until endSqN is the SqN of the instr you want to stop
  //replaying.
  val misprReplay_c = WireDefault(misprReplay_r)

  when(io.IN_branch.taken) {
    misprReplay_c.valid   := true.B
    misprReplay_c.endSqN  := io.IN_branch.sqN.value
    misprReplay_c.iterSqN := baseIndex
  }

  val misprReplayEnd = misprReplay_c.valid &&
    (misprReplay_c.iterSqN.value + WIDTH.U - 1.U >= misprReplay_c.endSqN .value||
      (misprReplay_c.iterSqN.value + WIDTH.U - 1.U).asSInt -
        misprReplay_c.endSqN.value.asSInt === 0.S)

  // Sequential update
  misprReplay_r := misprReplay_c
  when(misprReplay_c.valid) {
    when(misprReplayEnd) {
      misprReplay_r.valid := false.B
    }.otherwise {
      misprReplay_r.iterSqN := misprReplay_c.iterSqN.value + WIDTH.U
    }
  }

  val misprReplayFwdMask = Wire(Vec(WIDTH , Bool()))

  for (i <- 0 until WIDTH) {
    val curSqN = misprReplay_c.iterSqN.value + i.U
    misprReplayFwdMask(i) := (curSqN.asSInt - misprReplay_c.endSqN.value.asSInt) <= 0.S

  }

  misprReplayEnd := !misprReplayFwdMask(WIDTH - 1) ||
    (misprReplay_c.iterSqN.value + (WIDTH - 1).U) === misprReplay_c.endSqN.value
  io.OUT_mispredFlush := RegNext(misprReplay_c.valid && misprReplayFwdMask.asUInt.orR)

  when(misprReplay_c.valid) {
    for (i <- 0 until WIDTH) {
      when(misprReplayFwdMask(i)) {
        // Read entry at iterSqN + i
        val replayAddr = misprReplay_c.iterSqN.value + i.U
        val replayBank = replayAddr(log2Ceil(WIDTH)-1, 0)
        val replayIdx  = replayAddr(ID_LEN-1, log2Ceil(WIDTH))

        // Drive commit port for rename table rollback
        // rd comes from entry, tag comes from entry
        // On trap entries rd is redirected to 0
        io.OUT_comUOp(i).valid  := true.B
        io.OUT_comUOp(i).rd     := MuxLookup(replayBank, 0.U.asTypeOf(new ROBEntry))(
          (0 until WIDTH).map(b =>
            b.U -> entries(b)(replayIdx))
        ).rd
        io.OUT_comUOp(i).tagDst := MuxLookup(replayBank, 0.U.asTypeOf(new ROBEntry))(
          (0 until WIDTH).map(b =>
            b.U -> entries(b)(replayIdx))
        ).tag
      }
    }
  }

  //Hang Detection
  // Counts cycles without a commit. On overflow forces a timeout commit
  // via timeoutCommit in Step 6 which trips didCommit and resets the counter.

  when(reset.asBool) {
    hangCounter  := 0.U
    hangDetected := false.B
  }.elsewhen(didCommit) {
    // Something committed this cycle — pipeline is not hung, reset counter
    hangCounter  := 0.U
    hangDetected := false.B
  }.elsewhen(!hangDetected) {
    // No commit and not yet detected — increment counter
    // +& preserves the carry bit to detect overflow
    val hangInc  = hangCounter +& 1.U
    hangDetected := hangInc(HangConst.HANG_COUNTER_LEN)
    hangCounter  := hangInc(HangConst.HANG_COUNTER_LEN - 1, 0)
  }

}