package branch_pred

import chisel3._
import chisel3.util._
import branch_pred.branchParams._

// 2-bit saturating counter based branch prediction table.
// IDX_LEN controls the number of entries (2^IDX_LEN counters).
class BranchPredictionTable(params: branchParams)(IDX_LEN: Int = params.BP_BASEP_ID_LEN) extends Module {

  val io = IO(new Bundle {
    val readValid  = Input(Bool())
    val readAddr   = Input(UInt(IDX_LEN.W))
    val taken      = Output(Bool())

    val write_en   = Input(Bool())
    val writeAddr  = Input(UInt(IDX_LEN.W))
    val writeInit  = Input(Bool())
    val writeTaken = Input(Bool())
  })

  val numCounters = 1 << IDX_LEN
  val pred = RegInit(VecInit(Seq.fill(numCounters)(false.B)))
  val hist = RegInit(VecInit(Seq.fill(numCounters)(false.B)))

  // ---- Read path ---------------------------------------------
  io.taken := false.B
  when(io.readValid) {
    io.taken := pred(io.readAddr)
  }

  // ---- Write bundle ------------------------------------------
  // The write is pipelined one cycle to allow the read of the
  // current counter value to settle before the update.
  class WriteBundle extends Bundle {
    val taken = Bool()
    val init  = Bool()
    val addr  = UInt(IDX_LEN.W)
    val valid = Bool()
  }

  val writeTempReg = RegInit(0.U(2.W))
  val write_c      = Wire(new WriteBundle)
  val write_r      = RegInit(0.U.asTypeOf(new WriteBundle))

  write_c.valid  := io.write_en
  write_c.init   := io.writeInit
  write_c.addr   := io.writeAddr
  write_c.taken  := io.writeTaken

  // ---- Reset sequencer ---------------------------------------
  // Clears all counters on startup one entry per cycle.
  val resetIdx = RegInit(0.U((IDX_LEN + 1).W))

  when(!resetIdx(IDX_LEN)) {
    pred(resetIdx(IDX_LEN - 1, 0)) := false.B
    hist(resetIdx(IDX_LEN - 1, 0)) := false.B
    resetIdx := resetIdx + 1.U
  }.otherwise {
    write_r := write_c

    when(write_c.valid) {
      writeTempReg := Cat(pred(write_c.addr), hist(write_c.addr))
    }

    when(write_r.valid) {
      val nextVal = WireInit(writeTempReg)

      when(write_r.init) {
        nextVal := Cat(write_r.taken, !write_r.taken)
      }.elsewhen(writeTempReg =/= 3.U && write_r.taken) {
        nextVal := writeTempReg + 1.U
      }.elsewhen(writeTempReg =/= 0.U && !write_r.taken) {
        nextVal := writeTempReg - 1.U
      }

      pred(write_r.addr) := nextVal(1)
      hist(write_r.addr) := nextVal(0)
    }
  }
}
