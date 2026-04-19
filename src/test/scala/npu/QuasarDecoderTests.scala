package npu

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec

/** Wraps the pure combinational QuasarDecoder.decode() in a Module for testing */
class QuasarDecoderDUT extends Module {
  val io = IO(new Bundle {
    val inst    = Input(UInt(32.W))
    val decoded = Output(new QnpuDecoded)
  })
  io.decoded := QuasarDecoder.decode(io.inst)
}

class QuasarDecoderTests extends AnyFlatSpec with ChiselScalatestTester {

  behavior.of("QuasarDecoder")

  // helper: build a custom-0 R-type instruction word
  def mkInst(funct7: Int, rs2: Int, rs1: Int, funct3: Int, rd: Int): Long = {
    ((funct7 & 0x7f).toLong << 25) |
      ((rs2    & 0x1f).toLong << 20) |
      ((rs1    & 0x1f).toLong << 15) |
      ((funct3 & 0x07).toLong << 12) |
      ((rd     & 0x1f).toLong <<  7) |
      0x0b.toLong  // custom-0 opcode
  }

  it should "decode DISPATCH correctly" in {
    test(new QuasarDecoderDUT) { dut =>
      dut.io.inst.poke(mkInst(0, 0, 0, 0, 0).U)
      dut.clock.step(1)
      dut.io.decoded.valid.expect(true.B)
      dut.io.decoded.dispatch.expect(true.B)
      dut.io.decoded.poll.expect(false.B)
      dut.io.decoded.fence.expect(false.B)
    }
  }

  it should "decode POLL correctly and capture rd" in {
    test(new QuasarDecoderDUT) { dut =>
      dut.io.inst.poke(mkInst(0, 0, 0, 1, 5).U)  // rd = x5
      dut.clock.step(1)
      dut.io.decoded.valid.expect(true.B)
      dut.io.decoded.poll.expect(true.B)
      dut.io.decoded.dispatch.expect(false.B)
      dut.io.decoded.rd.expect(5.U)
    }
  }

  it should "decode FENCE correctly" in {
    test(new QuasarDecoderDUT) { dut =>
      dut.io.inst.poke(mkInst(0, 0, 0, 2, 0).U)
      dut.clock.step(1)
      dut.io.decoded.valid.expect(true.B)
      dut.io.decoded.fence.expect(true.B)
      dut.io.decoded.dispatch.expect(false.B)
      dut.io.decoded.poll.expect(false.B)
    }
  }

  it should "not decode a standard RISC-V instruction as QNPU" in {
    test(new QuasarDecoderDUT) { dut =>
      // ADDI x1, x0, 1 — opcode = 0x13, not 0x0B
      dut.io.inst.poke("h00100093".U)
      dut.clock.step(1)
      dut.io.decoded.valid.expect(false.B)
      dut.io.decoded.dispatch.expect(false.B)
      dut.io.decoded.poll.expect(false.B)
      dut.io.decoded.fence.expect(false.B)
    }
  }

  it should "not decode unknown funct3 as valid" in {
    test(new QuasarDecoderDUT) { dut =>
      dut.io.inst.poke(mkInst(0, 0, 0, 7, 0).U)  // funct3=7, undefined
      dut.clock.step(1)
      dut.io.decoded.valid.expect(false.B)
    }
  }
}