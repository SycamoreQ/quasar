package npu

import chisel3._
import chisel3.util._

/**
 * RISC-V custom-0 opcode space: bits[6:0] = 0x0B
 *
 * We define 3 NPU instructions, all R-type encoded:
 *
 *   QNPU.DISPATCH  funct7=0x00, funct3=0x0
 *     — triggers NPU execution (equivalent to writing QSR_CTRL bit 0)
 *     — rs1: ignored, rs2: ignored, rd: ignored
 *
 *   QNPU.POLL      funct7=0x00, funct3=0x1
 *     — reads QSR_STATUS into rd
 *     — non-blocking: returns current status immediately
 *
 *   QNPU.FENCE     funct7=0x00, funct3=0x2
 *     — stalls the host pipeline until NPU busy == 0
 *     — rd: 0 when NPU is done (written back to regfile)
 *
 * Encoding summary (32-bit instruction word):
 *   [31:25] funct7  = 0b0000000
 *   [24:20] rs2     = 0b00000 (unused)
 *   [19:15] rs1     = 0b00000 (unused)
 *   [14:12] funct3  = 0x0 / 0x1 / 0x2
 *   [11:7]  rd      = destination (used by POLL)
 *   [6:0]   opcode  = 0x0B
 */

object QnpuOpcode {
  val CUSTOM_0 = "b0001011".U(7.W)  // custom-0 opcode
}

object QnpuFunct3 {
  val DISPATCH = 0x0.U(3.W)
  val POLL     = 0x1.U(3.W)
  val FENCE    = 0x2.U(3.W)
}

/** Decoded NPU instruction signals, consumed by the host datapath */
class QnpuDecoded extends Bundle {
  val valid    = Bool()  // true if this is any QNPU instruction
  val dispatch = Bool()  // QNPU.DISPATCH
  val poll     = Bool()  // QNPU.POLL
  val fence    = Bool()  // QNPU.FENCE
  val rd       = UInt(5.W)
}

object QuasarDecoder {
  /**
   * Decode a 32-bit instruction word into a QnpuDecoded bundle.
   */
  def decode(inst: UInt): QnpuDecoded = {
    val decoded = Wire(new QnpuDecoded)

    val opcode = inst(6, 0)
    val funct3 = inst(14, 12)
    val rd = inst(11, 7)

    val isCustom0 = opcode === QnpuOpcode.CUSTOM_0
    val knownFunct3 = (funct3 === QnpuFunct3.DISPATCH) ||
      (funct3 === QnpuFunct3.POLL)     ||
      (funct3 === QnpuFunct3.FENCE)

    decoded.valid := isCustom0 && knownFunct3
    decoded.dispatch := isCustom0 && (funct3 === QnpuFunct3.DISPATCH)
    decoded.poll := isCustom0 && (funct3 === QnpuFunct3.POLL)
    decoded.fence := isCustom0 && (funct3 === QnpuFunct3.FENCE)
    decoded.rd := rd

    decoded
  }
}