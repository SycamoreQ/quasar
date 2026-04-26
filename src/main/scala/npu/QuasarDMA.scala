package npu

import chisel3._
import chisel3.util._
import junctions._

/**
 * QuasarDMAParams: configuration for the DMA engine.
 *
 * @param maxBurstLen   maximum AXI4 burst length in beats
 * @param dataWidth     AXI data bus width in bits (must match NASti)
 * @param addrWidth     AXI address width
 */
case class QuasarDMAParams(
                            maxBurstLen: Int = 16,
                            dataWidth:   Int = 64,
                            addrWidth:   Int = 32
                          )

/**
 * QuasarDMAIO: control and data interface for the DMA engine.
 *
 * Control interface (from tiling controller):
 *   src_addr   — AXI byte address to read from (external memory)
 *   dst_addr   — scratchpad word address to write to
 *   length     — number of scratchpad words to transfer
 *   start      — pulse high for one cycle to begin transfer
 *   done       — pulses high for one cycle when transfer completes
 *   busy       — level signal, high while transfer in progress
 *
 * Memory interface:
 *   axi        — AXI4 master port (connects to NASti bus)
 *
 * Scratchpad interface:
 *   sp_port    — write port into the scratchpad double buffer
 */
class QuasarDMAIO(
                   p:     QuasarDMAParams,
                   sp:    QuasarScratchpadParams,
                   nasti: NastiBundleParameters
                 ) extends Bundle {
  val src_addr = Input(UInt(p.addrWidth.W))
  val dst_addr = Input(UInt(sp.addrBits.W))
  val length   = Input(UInt(16.W))
  val start    = Input(Bool())
  val done     = Output(Bool())
  val busy     = Output(Bool())
  val axi      = new NastiBundle(nasti)
  val sp_port  = Flipped(new ScratchpadWritePort(sp))  // ← updated type
}

/**
 * QuasarDMA: AXI4 burst read DMA engine.
 */
class QuasarDMA(
                 val p:     QuasarDMAParams        = QuasarDMAParams(),
                 val sp:    QuasarScratchpadParams = QuasarScratchpadParams(),
                 val nasti: NastiBundleParameters  = NastiBundleParameters(32, 64, 5)
               ) extends Module {
  val io = IO(new QuasarDMAIO(p, sp, nasti))

  object State extends ChiselEnum {
    val sIdle, sAddr, sData, sDone = Value
  }
  import State._

  val state = RegInit(sIdle)
  val src_addr_reg = RegInit(0.U(p.addrWidth.W))
  val dst_addr_reg = RegInit(0.U(sp.addrBits.W))
  val remaining = RegInit(0.U(16.W))

  io.axi.aw.valid := false.B
  io.axi.aw.bits := 0.U.asTypeOf(io.axi.aw.bits)
  io.axi.w.valid := false.B
  io.axi.w.bits := 0.U.asTypeOf(io.axi.w.bits)
  io.axi.b.ready := false.B
  io.axi.ar.valid := false.B
  io.axi.ar.bits := 0.U.asTypeOf(io.axi.ar.bits)
  io.axi.r.ready := false.B

  io.sp_port.en := false.B
  io.sp_port.addr := dst_addr_reg
  io.sp_port.data := 0.U
  io.done := false.B
  io.busy := state =/= sIdle

  //Burst length calculation
  // Each AXI beat is 8 bytes (64-bit bus)
  // Each scratchpad word is 4 bytes
  // So 2 scratchpad words per beat
  // Burst length = ceil(remaining / 2)
  val beats     = (remaining + 1.U) >> 1
  val maxBurst  = p.maxBurstLen.U(16.W)
  val burst_len = Mux(
    remaining > p.maxBurstLen.U,
    (p.maxBurstLen - 1).U(8.W),
    (remaining - 1.U)(7, 0)
  )

  switch(state) {

    is(sIdle) {
      when(io.start) {
        src_addr_reg := io.src_addr
        dst_addr_reg := io.dst_addr
        remaining := io.length
        state := sAddr
      }
    }

    is(sAddr) {
      // Drive read address channel
      io.axi.ar.valid := true.B
      io.axi.ar.bits.addr := src_addr_reg
      io.axi.ar.bits.len := burst_len
      io.axi.ar.bits.size  := 3.U  // 8 bytes per beat
      io.axi.ar.bits.burst  := NastiConstants.BurstIncr
      io.axi.ar.bits.id := 0.U
      io.axi.ar.bits.lock  := 0.U
      io.axi.ar.bits.cache := 0.U
      io.axi.ar.bits.prot := 0.U
      io.axi.ar.bits.qos := 0.U

      when(io.axi.ar.fire) {
        state := sData
      }
    }

    is(sData) {
      io.axi.r.ready := true.B

      when(io.axi.r.valid) {
        // Write lower 32 bits of 64-bit beat to scratchpad
        io.sp_port.en := true.B
        io.sp_port.addr := dst_addr_reg
        io.sp_port.data := io.axi.r.bits.data(31, 0)

        dst_addr_reg := dst_addr_reg + 1.U
        src_addr_reg := src_addr_reg + 8.U
        remaining := remaining - 1.U

        when(io.axi.r.bits.last) {
          when(remaining <= 1.U) {
            state := sDone
          }.otherwise {
            state := sAddr  // more data to fetch, issue next burst
          }
        }
      }
    }

    is(sDone) {
      io.done := true.B
      state := sIdle
    }
  }
}