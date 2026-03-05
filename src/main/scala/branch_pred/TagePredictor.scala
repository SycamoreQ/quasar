package branch_pred

import chisel3._
import chisel3.util._

class TagePredictor(params: branchParams)(
  NUM_STAGES : Int = params.TAGE_STAGES,
  FACTOR     : Int = 2,
  BASE       : Int = 2,
  TABLE_SIZE : Int = 256,
  TAG_SIZE   : Int = 9
) extends Module {

  val HASH_SIZE = log2Ceil(TABLE_SIZE)

  val io = IO(new Bundle {
    val predValid   = Input(Bool())
    val predHistory = Input(new BHist)
    val predAddr    = Input(UInt(31.W))

    val predTageID  = Output(new TAGEID)
    val altPred     = Output(Bool())
    val predTaken   = Output(Bool())

    val writeAddr    = Input(UInt(31.W))
    val writeValid   = Input(Bool())
    val writeHistory = Input(new BHist)
    val writeTageID  = Input(new TAGEID)
    val writeTaken   = Input(Bool())
    val writeAltPred = Input(Bool())
    val writePred    = Input(Bool())
  })

  val valid       = Wire(Vec(NUM_STAGES, Bool()))
  val predictions = Wire(Vec(NUM_STAGES, Bool()))

  //  Base predictor — 2-bit saturating counter table
  val basePredictor = Module(new BranchPredictionTable(params)(params.BP_BASEP_ID_LEN))
  basePredictor.io.readValid  := io.predValid
  basePredictor.io.readAddr   := io.predAddr(params.BP_BASEP_ID_LEN - 1, 0)
  predictions(0)              := basePredictor.io.taken
  basePredictor.io.write_en   := io.writeValid
  basePredictor.io.writeAddr  := io.writeAddr(params.BP_BASEP_ID_LEN - 1, 0)
  basePredictor.io.writeInit  := false.B
  basePredictor.io.writeTaken := io.writeTaken

  valid(0) := true.B

  //  Hash and tag computation for each TAGE stage
  //  predHashes / writeHashes : index into each TAGE table
  //  predTags   / writeTags   : tag bits for match checking
  //
  //  These are Wires because they are purely combinational
  //  functions of the current PC and history.
  val predHashes  = Wire(Vec(NUM_STAGES - 1, UInt(HASH_SIZE.W)))
  val writeHashes = Wire(Vec(NUM_STAGES - 1, UInt(HASH_SIZE.W)))
  val predTags    = Wire(Vec(NUM_STAGES - 1, UInt(TAG_SIZE.W)))
  val writeTags   = Wire(Vec(NUM_STAGES - 1, UInt(TAG_SIZE.W)))

  for (i <- 0 until NUM_STAGES - 1) {
    val hist_bits = BASE * math.pow(FACTOR, i).toInt

    predTags(i)    := io.predAddr(TAG_SIZE - 1, 0)
    writeTags(i)   := io.writeAddr(TAG_SIZE - 1, 0)
    predHashes(i)  := 0.U
    writeHashes(i) := 0.U

    // XOR fold the PC across HASH_SIZE-wide windows
    for (j <- 0 until io.predAddr.getWidth / HASH_SIZE) {
      predHashes(i)  := predHashes(i)  ^ io.predAddr (j * HASH_SIZE + HASH_SIZE - 1, j * HASH_SIZE)
      writeHashes(i) := writeHashes(i) ^ io.writeAddr(j * HASH_SIZE + HASH_SIZE - 1, j * HASH_SIZE)
    }

    // Mix in branch history bits
    for (j <- 0 until hist_bits) {
      predHashes(i)  := predHashes(i)  ^ (io.predHistory(j)  << (j % HASH_SIZE.U))
      writeHashes(i) := writeHashes(i) ^ (io.writeHistory(j) << (j % HASH_SIZE.U))
      predTags(i)    := predTags(i)    ^ ((io.predHistory(j)  ^ io.predHistory ((j + 1) % hist_bits)) << (j % TAG_SIZE.U))
      writeTags(i)   := writeTags(i)   ^ ((io.writeHistory(j) ^ io.writeHistory((j + 1) % hist_bits)) << (j % TAG_SIZE.U))
    }
  }

  //  LFSR for randomised allocation
  val random = LFSR(8)

  //  Allocation availability per stage
  val avail = Wire(Vec(NUM_STAGES, Bool()))
  avail(0) := true.B

  val followingAllocAvail = Wire(Vec(NUM_STAGES, Bool()))
  val doAlloc             = WireInit(VecInit(Seq.fill(NUM_STAGES)(false.B)))
  val allocFailed         = WireInit(false.B)

  for (i <- 0 until NUM_STAGES) {
    followingAllocAvail(i) := false.B
    for (j <- i + 1 until NUM_STAGES) {
      followingAllocAvail(i) := followingAllocAvail(i) || avail(j)
    }
  }

  // On a misprediction try to allocate a new entry in a stage
  // longer than the one that made the prediction.
  when(io.writeTaken =/= io.writePred) {
    var allocated = false.B
    for (i <- 0 until NUM_STAGES) {
      when(i.U > io.writeTageID.value &&
           avail(i) &&
           !allocated &&
           (!followingAllocAvail(i) ||
             random((i % 4) * 2 + 1, (i % 4) * 2) =/= 0.U)) {
        allocated    = true.B
        doAlloc(i)  := true.B
      }
    }
    allocFailed := !allocated
  }

  //  TAGE table stages
  val tageTables = for (i <- 1 until NUM_STAGES) yield {
    val tage = Module(new TageTable(TABLE_SIZE, TAG_SIZE))
    tage.io.readValid    := io.predValid
    tage.io.readAddr     := predHashes(i - 1)
    tage.io.readTag      := predTags(i - 1)
    valid(i)             := tage.io.readValid_out
    predictions(i)       := tage.io.readTaken
    tage.io.writeValid   := io.writeValid
    tage.io.writeAddr    := writeHashes(i - 1)
    tage.io.writeTag     := writeTags(i - 1)
    tage.io.writeTaken   := io.writeTaken
    tage.io.writeUpdate  := i.U === io.writeTageID.value
    tage.io.writeUseful  := io.writePred =/= io.writeAltPred
    tage.io.writeCorrect := io.writePred === io.writeTaken
    avail(i)             := tage.io.allocAvail
    tage.io.doAlloc      := doAlloc(i)
    tage.io.allocFailed  := allocFailed && i.U > io.writeTageID.value
    tage
  }

  //  Output — highest valid stage wins
  io.altPred          := predictions(0)
  io.predTaken        := predictions(0)
  io.predTageID.value := 0.U

  for (i <- 0 until NUM_STAGES) {
    when(valid(i)) {
      io.predTageID.value := i.U
      io.altPred          := io.predTaken
      io.predTaken        := predictions(i)
    }
  }
}
