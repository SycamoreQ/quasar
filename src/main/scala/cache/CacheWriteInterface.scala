package cache

import chisel3._
import chisel3.util._
import memory.AXI4Bundle
import memory.MemArbiter
import fetch.iFetchParams


class CacheWriteInterface(
                         ADDR_BITS: Int = 12, //CACHE_SIZE_E = 14 in iFetchParams
                         IWIDTH: Int = 64, //incoming data width
                         CWIDTH : Int = 32, //cache SRAM word width
                         ID_LEN: Int  = 2  //transaction ID width usually as log2(AXI_NUM_TRANS)
                         ) extends Module {

  val WNUM = IWIDTH/CWIDTH //sub-words when splitting (IWIDTH > CWIDTH)
  val CNUM = CWIDTH/IWIDTH //chunks to accumulate (IWIDTH < CWIDTH)
  val WIDTH = scala.math.max(IWIDTH , CWIDTH)
  val WM_LEN = scala.math.max(1 , CNUM)  //write mask width or chunk mask
  val CHUNK_LEN = log2Ceil(CWIDTH/IWIDTH) //bits needed to index chunks

  
}


