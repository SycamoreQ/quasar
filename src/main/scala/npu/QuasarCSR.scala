package npu

import chisel3._
import chisel3.util._

/**
 * QuasarCSR defines all NPU-visible control/status register addresses
 * and the register file that backs them.
 *
 * Address map (all in the custom RISC-V CSR space 0x800–0x8FF):
 *
 *   QSR_CTRL       0x800  — control register (write)
 *                           bit 0: dispatch (pulse) — start NPU execution
 *                           bit 1: reset NPU state
 *                           bits 31:2 reserved
 *
 *   QSR_STATUS     0x801  — status register (read-only)
 *                           bit 0: busy (1 = NPU executing)
 *                           bit 1: done (1 = last dispatch completed)
 *                           bit 2: error
 *                           bits 31:3 reserved
 *
 *   QSR_BASE_ADDR  0x802  — base address of input tensor in scratchpad
 *   QSR_OUT_ADDR   0x803  — base address of output tensor in scratchpad
 *   QSR_SHAPE      0x804  — packed tensor shape: bits[15:0]=M, bits[31:16]=N
 *   QSR_CMD        0x805  — operation code
 *                           0x0 = NOP
 *                           0x1 = GEMM
 *                           0x2 = RELU
 *                           0x3 = GELU (stub, Phase 6)
 *                           0x4 = ATTENTION (stub, Phase 6)
 */

object QuasarCSRAddr {
  val QSR_CTRL      = 0x800.U(12.W)
  val QSR_STATUS    = 0x801.U(12.W)
  val QSR_BASE_ADDR = 0x802.U(12.W)
  val QSR_OUT_ADDR  = 0x803.U(12.W)
  val QSR_SHAPE     = 0x804.U(12.W)
  val QSR_CMD       = 0x805.U(12.W)

  // Convenience: all NPU CSR addresses as a Seq for address decode
  val all = Seq(
    QSR_CTRL, QSR_STATUS, QSR_BASE_ADDR,
    QSR_OUT_ADDR, QSR_SHAPE, QSR_CMD
  )
  def isNpuCSR(addr: UInt): Bool = {
    (addr >= 0x800.U) && (addr <= 0x805.U)
  }

  // VERSION B: For Software (used in your Tests)
  def isNpuCSR(addr: Int): Boolean = {
    addr >= 0x800 && addr <= 0x805
  }
}

object QuasarCmd {
  val NOP       = 0x0.U(4.W)
  val GEMM      = 0x1.U(4.W)
  val RELU      = 0x2.U(4.W)
  val GELU      = 0x3.U(4.W)  // stub
  val ATTENTION = 0x4.U(4.W)  // stub
}

/** IO bundle exposed by the CSR file to the rest of the NPU */
class QuasarCSRIO(xlen: Int) extends Bundle {
  // from host CPU write path
  val wen   = Input(Bool())
  val waddr = Input(UInt(12.W))
  val wdata = Input(UInt(xlen.W))

  // from host CPU read path
  val raddr = Input(UInt(12.W))
  val rdata = Output(UInt(xlen.W))

  // status inputs driven by NPU datapath
  val npu_busy  = Input(Bool())
  val npu_done  = Input(Bool())
  val npu_error = Input(Bool())

  // decoded outputs consumed by NPU datapath
  val dispatch  = Output(Bool())  // single-cycle pulse when bit 0 of CTRL written
  val npu_reset = Output(Bool())  // level signal from CTRL bit 1
  val base_addr = Output(UInt(xlen.W))
  val out_addr  = Output(UInt(xlen.W))
  val shape_m   = Output(UInt(16.W))
  val shape_n   = Output(UInt(16.W))
  val cmd       = Output(UInt(4.W))
}

/**
 * QuasarCSRFile: the register file backing the NPU CSR address map.
 *
 * Registers:
 *   ctrl_reg      — written by host, cleared dispatch pulse after one cycle
 *   status_reg    — read-only, driven by npu_busy/done/error inputs
 *   base_addr_reg — latched from wdata when waddr == QSR_BASE_ADDR
 *   out_addr_reg  — latched from wdata when waddr == QSR_OUT_ADDR
 *   shape_reg     — latched from wdata when waddr == QSR_SHAPE
 *   cmd_reg       — latched from wdata when waddr == QSR_CMD
 */
class QuasarCSRFile(xlen: Int) extends Module {
  val io = IO(new QuasarCSRIO(xlen))

  val ctrl_reg      = RegInit(0.U(xlen.W))
  val base_addr_reg = RegInit(0.U(xlen.W))
  val out_addr_reg  = RegInit(0.U(xlen.W))
  val shape_reg     = RegInit(0.U(xlen.W))
  val cmd_reg       = RegInit(0.U(xlen.W))

  when(io.wen) {
    when(io.waddr === QuasarCSRAddr.QSR_CTRL) {
      ctrl_reg := io.wdata
    }.elsewhen(io.waddr === QuasarCSRAddr.QSR_BASE_ADDR) {
      base_addr_reg := io.wdata
    }.elsewhen(io.waddr === QuasarCSRAddr.QSR_OUT_ADDR) {
      out_addr_reg := io.wdata
    }.elsewhen(io.waddr === QuasarCSRAddr.QSR_SHAPE){
      shape_reg := io.wdata
    }.elsewhen(io.waddr === QuasarCSRAddr.QSR_CMD){
      cmd_reg := io.wdata
    }
  }

  ctrl_reg := Cat(ctrl_reg(xlen-1, 1), 0.U(1.W))
  when(io.wen && io.waddr === QuasarCSRAddr.QSR_CTRL) {
    ctrl_reg := io.wdata
  }


  val status_wire = Cat(0.U(29.W), io.npu_error, io.npu_done, io.npu_busy)

  io.rdata := MuxLookup(io.raddr, 0.U)(Seq(
    QuasarCSRAddr.QSR_CTRL      -> ctrl_reg,
    QuasarCSRAddr.QSR_STATUS    -> status_wire,
    QuasarCSRAddr.QSR_BASE_ADDR -> base_addr_reg,
    QuasarCSRAddr.QSR_OUT_ADDR  -> out_addr_reg,
    QuasarCSRAddr.QSR_SHAPE     -> shape_reg,
    QuasarCSRAddr.QSR_CMD       -> cmd_reg,
  ))

  io.dispatch := ctrl_reg(0)
  io.npu_reset := ctrl_reg(1)
  io.base_addr := base_addr_reg
  io.out_addr := out_addr_reg
  io.shape_m := shape_reg(15, 0)
  io.shape_n := shape_reg(31, 16)
  io.cmd := cmd_reg(3, 0)

}