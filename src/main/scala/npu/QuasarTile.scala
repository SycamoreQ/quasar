package npu

import chisel3._
import chisel3.util._
import mini._
import junctions._

/**
 * QuasarTile is the top-level module for Phase 0.
 * It instantiates:
 *   - the riscv-mini Tile (host CPU + caches + memory arbiter)
 *   - the QuasarCSRFile (NPU register file)
 *   - a stub QuasarNpuTop (the NPU datapath — just wires for now)
 *
 * In Phase 0 the NPU datapath does nothing. The tile simply proves
 * that the CSR file elaborates, connects to the host, and that
 * the custom instruction decoder produces correct signals.
 *
 * IO is identical to mini.Tile plus one debug port for testability.
 */
class QuasarTileIO(xlen: Int, nastiParams: NastiBundleParameters) extends Bundle {
  val host  = new HostIO(xlen)
  val nasti = new NastiBundle(nastiParams)
  // debug: expose NPU status so tests can observe it without CSR reads
  val npu_status = Output(UInt(3.W))  // {error, done, busy}
}

/**
 * Stub NPU top todo() — just holds the CSR file and ties off the datapath.
 * Replace this progressively from Phase 1 onward.
 */
class QuasarNpuTop(xlen: Int) extends Module {
  val io = IO(new Bundle {
    val csr = new QuasarCSRIO(xlen)
  })

  io.csr.npu_busy  := false.B
  io.csr.npu_done  := true.B
  io.csr.npu_error := false.B
}

class QuasarTile(
                  val coreParams:  CoreConfig,
                  val nastiParams: NastiBundleParameters,
                  val cacheParams: CacheConfig
                ) extends Module {

  val io    = IO(new QuasarTileIO(coreParams.xlen, nastiParams))
  val xlen  = coreParams.xlen

  val host_tile = Module(new Tile(coreParams, nastiParams, cacheParams))
  val csr_file  = Module(new QuasarCSRFile(xlen))
  val npu_top   = Module(new QuasarNpuTop(xlen))

  io.host  <> host_tile.io.host
  io.nasti <> host_tile.io.nasti

  csr_file.io.npu_busy  := npu_top.io.csr.npu_busy
  csr_file.io.npu_done := npu_top.io.csr.npu_done
  csr_file.io.npu_error := npu_top.io.csr.npu_error

  //todo: tie off host write/read ports for now (Phase 5 wires these for real)
  csr_file.io.wen := false.B
  csr_file.io.waddr := 0.U
  csr_file.io.wdata := 0.U
  csr_file.io.raddr := 0.U


  io.npu_status := Cat(csr_file.io.npu_error,
    csr_file.io.npu_done,
    csr_file.io.npu_busy)

  //npu_top gets its status outputs from itself
  // and the CSR file drives the control outputs to npu_top
  // but since npu_top is a stub we just tie the remaining ports
  npu_top.io.csr.dispatch  := csr_file.io.dispatch
  npu_top.io.csr.npu_reset := csr_file.io.npu_reset
  npu_top.io.csr.base_addr := csr_file.io.base_addr
  npu_top.io.csr.out_addr  := csr_file.io.out_addr
  npu_top.io.csr.shape_m   := csr_file.io.shape_m
  npu_top.io.csr.shape_n   := csr_file.io.shape_n
  npu_top.io.csr.cmd       := csr_file.io.cmd


}