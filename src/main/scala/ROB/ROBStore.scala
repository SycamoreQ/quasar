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

  val isRenamed = Wire(Bool())  // the instruction has been dispatched
  val isExecuted = Wire(Bool()) //The execution unit has written back a result.
  val noFlagsConflict = Wire(Bool())
  val sqAllowCommit = Wire(Bool())
  val timeOutCommit = Wire(Bool())

  for (i <- 0 until WIDTH){
    isRenamed := DontCare
    isExecuted := deqFlags(i) =/= Flags.FLAGS_NX
    noFlagsConflict := !io.IN_ldComLimit.valid || $signed(loadSqN_r - io.IN_ldComLimit.sqN.value) < 0
    sqAllowCommit := !io.IN_ldComLimit.valid || $signed(loadSqN_r - io.IN_stComLimit(i).sqN.value) < 0
  }




}