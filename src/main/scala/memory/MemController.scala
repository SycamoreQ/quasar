package memory

import chisel3._
import chisel3.util._
import fetch.iFetchParams

/*
MemController is the bridge between your chip's internal bus and physical DRAM.
Every module that needs to read or write memory — ICache, PageWalker, later DCache — sends requests to the MemController.
It serializes them, handles DRAM timing, and returns responses.

The most important insight in the reference is that it does not use a simple single-request state machine.
Instead it maintains a transfer table —an array of in-flight transaction slots called transfers[AXI_NUM_TRANS].
Each slot tracks one independent transaction from enqueue to completion independently.

This means the MemController can have multiple transactions in flight simultaneously —
one might be waiting for an AXI read response while another is sending write data.
This is far more efficient than a sequential state machine that blocks on each transaction.

*/

class MemController(
                     NUM_TFS     : Int = 8,
                     NUM_TFS_IN  : Int = 3,
                     axiParams   : AXIParams,
                     fetchParams : iFetchParams
                   ) extends Module {

  val AXI_WIDTH = axiParams.AXI_WIDTH

  val io = IO(new Bundle {
    val IN_ctrl = Input(Vec(NUM_TFS_IN, new MemC_Req(fetchParams, AXI_WIDTH)))
    val OUT_stat = Output(new MemC_Res(fetchParams, NUM_TFS, NUM_TFS_IN, AXI_WIDTH))
    val axi = new AXI4Bundle(axiParams)

    // Cache SRAM interfaces wired to CacheWriteInterface / CacheReadInterface
    // These are the output ports that connect to the actual cache SRAMs
    val icacheW = Output(new CacheSRAMWriteIF(fetchParams))
    val dcacheW = Output(new CacheSRAMWriteIF(fetchParams)) // stub
    val dcacheR = Output(new CacheSRAMReadIF(fetchParams)) // stub
    val dcacheRData = Input(UInt((32 * 4).W)) // stub
    val clear = Input(Bool())
  })

  // Transfer table
  val transfers = RegInit(VecInit(
    Seq.fill(NUM_TFS)(0.U.asTypeOf(new Transfer(fetchParams, AXI_WIDTH)))
  ))

  //isMMIO — combinational per slot
  val isMMIO = VecInit(transfers.map { t =>
    t.valid && MuxLookup(t.cmd.asUInt, false.B)(Seq(
      MemC_Cmd.MEMC_READ_BYTE.asUInt -> true.B,
      MemC_Cmd.MEMC_READ_HALF.asUInt -> true.B,
      MemC_Cmd.MEMC_READ_WORD.asUInt -> true.B,
      MemC_Cmd.MEMC_WRITE_BYTE.asUInt -> true.B,
      MemC_Cmd.MEMC_WRITE_HALF.asUInt -> true.B,
      MemC_Cmd.MEMC_WRITE_WORD.asUInt -> true.B
    ))
  })


  //enqIdx — first free slot
  val enqIdxValid = transfers.map(!_.valid).reduce(_ || _)
  val enqIdx = PriorityEncoder(VecInit(transfers.map(!_.valid)).asUInt)

  def isCacheOp(cmd: MemC_Cmd.Type): Bool =
    cmd === MemC_Cmd.MEMC_REPLACE ||
      cmd === MemC_Cmd.MEMC_CP_CACHE_TO_EXT ||
      cmd === MemC_Cmd.MEMC_CP_EXT_TO_CACHE

  val selReq = WireDefault(0.U.asTypeOf(new MemC_Req(fetchParams, AXI_WIDTH)))
  val selPort = WireDefault(NUM_TFS_IN.U) // which port won, NUM_TFS_IN = none
  val stallVec = WireDefault(((1 << NUM_TFS_IN) - 1).U(NUM_TFS_IN.W))

  when(enqIdxValid) {
    for (i <- 0 until NUM_TFS_IN) {
      when(selPort === NUM_TFS_IN.U &&
        io.IN_ctrl(i).cmd =/= MemC_Cmd.MEMC_NONE) {

        // Collision check — no in-flight cache-line op on same line
        val coll = transfers.zipWithIndex.map { case (t, _) =>
          t.valid &&
            isCacheOp(t.cmd) &&
            isCacheOp(io.IN_ctrl(i).cmd) &&
            t.cacheID === io.IN_ctrl(i).cacheID &&
            t.cacheAddr(fetchParams.CACHE_SIZE_E-3, fetchParams.CLSIZE_E-2) ===
              io.IN_ctrl(i).cacheAddr(fetchParams.CACHE_SIZE_E-3, fetchParams.CLSIZE_E-2)
        }.reduce(_ || _)

        when(!coll) {
          selReq  := io.IN_ctrl(i)
          selPort := i.U
          stallVec := ~(1.U(NUM_TFS_IN.W) << i.U)
        }
      }
    }
  }

  io.OUT_stat.stall := stallVec
  io.OUT_stat.busy  := true.B
}


