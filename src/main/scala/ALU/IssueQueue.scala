package ALU

import ROB.TagConst.TAG_SIZE
import chisel3._
import chisel3.util._
import decoder.{BR_Op, FU_t, INT_Op}
import fetch.{BranchProv, iFetchParams}
import ROB.{ComLimit, Flags, FlagsUOp, R_UOp, RegFile, TagConst}
import branch_pred.SqN

/*
The issue queue collects types of instrs like multiplication/division , integer and sends it to the ALU.
*/

class IQEntry extends Bundle {
  val valid = Bool()
  val uop = new R_UOp(4)
  val availA = Bool()
  val availB = Bool()
  val availC = Bool()
}

class CDBEntry extends Bundle{
  val valid = Bool()
  val tag = UInt(TagConst.TAG_SIZE.W)
  val data = UInt()
}

class IssueQueue(WIDTH:Int = 8 , NUM_ENQ:Int = 4 , NUM_CDB:Int = 8)(params:iFetchParams ) extends Module{
  val io = IO(new Bundle {
    val in_stall = Input(Bool())
    val in_uop = Input(Vec(WIDTH, new R_UOp(4)))
    val in_enqValid = Input(Vec(WIDTH, Bool()))

    // From CDB — wakeup
    val in_cdb = Input(Vec(NUM_CDB, new CDBEntry))

    // From branch unit — flush
    val in_branch = Input(new BranchProv)

    // To execution unit — issue
    val out_uop = Output(new ExecUOp)
    val out_valid  = Output(Bool())

    // To register file — operand read request
    val out_rdA  = Output(UInt(TAG_SIZE.W))
    val out_rdB = Output(UInt(TAG_SIZE.W))

    // From register file — operand values
    val in_rdDataA= Input(UInt(32.W))
    val in_rdDataB = Input(UInt(32.W))

    // To Rename — backpressure
    val out_stall = Output(Bool())

    // AGUIQ only — load/store ordering
    val in_ldComLimit = Input(new ComLimit)
    val in_stComLimit= Input(new ComLimit)
  })

  val entry = RegInit(VecInit(Seq.fill(WIDTH)(0.U.asTypeOf(new IQEntry))))

  // Default
  io.out_stall := false.B

  val freeSlots = VecInit(entry.map(!_.valid)).asUInt

  // Count how many incoming uops need slots
  // Stall if not enough free slots
  val numEnq    = PopCount(io.in_enqValid)
  val numFree   = PopCount(freeSlots)
  io.out_stall  := numFree < numEnq
  var claimedMask = 0.U(WIDTH.W)

  for (i <- 0 until NUM_ENQ) {

    val availSlots = freeSlots & ~claimedMask
    val freeIdx = PriorityEncoder(freeSlots)
    when(io.in_enqValid(i) && io.in_uop(i).valid) {
      // Find free slot — each iteration needs its own PriorityEncoder
      // accounting for slots taken by earlier iterations
      when(freeSlots.orR) {
        entry(freeIdx).valid  := true.B
        entry(freeIdx).uop    := io.in_uop(i)
        entry(freeIdx).availA := io.in_uop(i).availA
        entry(freeIdx).availB := io.in_uop(i).availB
        entry(freeIdx).availC := io.in_uop(i).availC
        claimedMask = claimedMask | UIntToOH(freeIdx, WIDTH)
      }


    }
  }

  for (i <- 0 until WIDTH) {
    for (j <- 0 until NUM_CDB) {
      when (io.in_cdb(j).tag === entry(i).uop.tagA){
        entry(i).uop.availA := true.B
      }

      when (io.in_cdb(j).tag === entry(i).uop.tagB){
        entry(i).uop.availB := true.B
      }

      when (io.in_cdb(j).tag === entry(i).uop.tagC){
        entry(i).uop.availC := true.B
      }
    }
  }

  var bestIdx = 0.U(log2Ceil(WIDTH).W)
  var bestSqN = ~0.U(32.W)
  var found   = false.B

  // Compute the 16-bit readyMask
  val readyMask = VecInit((0 until 16).map { i =>
    entry(i).valid && entry(i).availA && entry(i).availB && entry(i).availC
  }).asUInt

  for (i <- 0 until WIDTH){
    when (readyMask(i) && entry(i).uop.sqN.value < bestSqN) {
      bestIdx = i.U
      bestSqN = entry(i).uop.sqN.value
      found  = true.B
    }
  }

  io.out_valid := false.B
  io.out_rdA   := 0.U
  io.out_rdB   := 0.U
  io.out_uop   := 0.U.asTypeOf(new ExecUOp)

  when (found) {
     for (i <- 0 until NUM_CDB){
       io.out_rdA := entry(bestIdx).uop.tagA(6, 0)  // strip MSB special flag
       io.out_rdB := entry(bestIdx).uop.tagB(6, 0)

       val execUop = WireDefault(0.U.asTypeOf(new ExecUOp))
       execUop.uop  := entry(bestIdx).uop
       execUop.srcA := Mux(entry(bestIdx).uop.immB,
         entry(bestIdx).uop.imm,
         io.in_rdDataA)
       execUop.srcB := io.in_rdDataB
       execUop.pc   := entry(bestIdx).uop.pc
       io.out_uop   := execUop

       io.out_valid := true.B
       entry(i).uop.valid := true.B

     }
  }

  when(io.in_branch.taken) {
    for (i <- 0 until WIDTH) {
      when((entry(i).uop.sqN.value.asSInt -
        io.in_branch.sqN.value.asSInt) > 0.S) {
        entry(i).valid := false.B
      }
    }
  }

  for (i <- 0 until NUM_ENQ){
    io.out_uop(i).pc := io.in_uop(i).pc
  }
}