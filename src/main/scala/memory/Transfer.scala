package memory

import chisel3._
import chisel3.util._
import fetch.iFetchParams

object MemC_Cmd extends ChiselEnum {
  val MEMC_NONE,
  MEMC_REPLACE,
  MEMC_CP_EXT_TO_CACHE,
  MEMC_CP_CACHE_TO_EXT,
  MEMC_READ_BYTE,
  MEMC_READ_HALF,
  MEMC_READ_WORD,
  MEMC_WRITE_BYTE,
  MEMC_WRITE_HALF,
  MEMC_WRITE_WORD = Value
}

class Transfer(params: iFetchParams, AXI_WIDTH: Int = 32) extends Bundle {

  val valid    = Bool()
  val cmd      = MemC_Cmd()
  val cacheID  = UInt(1.W)   // 0 = DCache, 1 = ICache

  val readAddr  = UInt(32.W)
  val writeAddr = UInt(32.W)
  val cacheAddr = UInt((params.CACHE_SIZE_E - 2).W)

  val progress       = UInt((params.CLSIZE_E - 1).W)
  val evictProgress  = UInt((params.CLSIZE_E - 1).W)
  val addrCounter    = UInt((params.CLSIZE_E - 1).W)
  val fwdAddrCounter = UInt((params.CLSIZE_E - 1).W)

  val needReadRq  = Bool()
  val needWriteRq = UInt(2.W)

  val readDone  = Bool()
  val writeDone = Bool()

  // Store fuse — for DCache store miss optimization
  val storeData = UInt(AXI_WIDTH.W)
  val storeMask = UInt((AXI_WIDTH / 8).W)
}