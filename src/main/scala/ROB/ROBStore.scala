package ROB

import chisel3._
import chisel3.util._
import fetch.{BPUpdate, BranchProv, FetchID, FetchOff, iFetchParams}
import branch_pred.SqN
import decoder.INT_Op

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
  val baseIndex = RegInit(0.U((ID_LEN + 1).W))  // commit head
  val lastIndex = RegInit(0.U((ID_LEN + 1).W))  // dispatch tail

  // Load/store SqN tracking at commit boundary
  val loadSqN_r  = RegInit(0.U((ID_LEN + 1).W))
  val storeSqN_r = RegInit(((1 << (ID_LEN + 1)) - 1).U((ID_LEN + 1).W)) // -1

  // Misprediction replay state
  val misprReplay_r = RegInit(0.U.asTypeOf(new MisprReplay))

  // Hang detection
  val hangCounter = RegInit(0.U(HangConst.HANG_COUNTER_LEN.W))
  val hangDetected = RegInit(false.B)
  val didCommit = RegInit(false.B)

  io.OUT_maxSqN := (baseIndex + (LENGTH - 1).U).asTypeOf(new SqN)
  io.OUT_curSqN := baseIndex.asTypeOf(new SqN)

  val rnUopSorted = WireDefault(Vec(WIDTH_RN , new R_UOp(4)))


  // sorting the uops to be in order and put them in a separate rename sorted buff
  for (i <- 0 until WIDTH_RN-1) {
    for (j <- 0 until WIDTH_RN-1) {
      when (io.IN_uop(j).valid && io.IN_uop(j).sqN.value ===i.U ){
        rnUopSorted(i).sqN := io.IN_uop(j).sqN
      }
    }

    when (rnUopSorted(i).valid === true.B && io.IN_branch.taken === false.B){
      val bankBits = log2Ceil(WIDTH_RN)
      val bankIndex = io.IN_uop(i).sqN.value(bankBits - 1, 0)
      val entryIdx  = io.IN_uop(i).sqN.value(ID_LEN - 1, bankBits)


    }
  }
}