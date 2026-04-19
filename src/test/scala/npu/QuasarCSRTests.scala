package npu

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec

class QuasarCSRTests extends AnyFlatSpec with ChiselScalatestTester {

  val xlen = 32

  behavior.of("QuasarCSRFile")

  // ── Test 1 ──────────────────────────────────────────────────────────────
  it should "reset to safe defaults" in {
    test(new QuasarCSRFile(xlen)) { dut =>
      dut.io.wen.poke(false.B)
      dut.io.waddr.poke(0.U)
      dut.io.wdata.poke(0.U)
      dut.io.raddr.poke(QuasarCSRAddr.QSR_STATUS)
      dut.io.npu_busy.poke(false.B)
      dut.io.npu_done.poke(false.B)
      dut.io.npu_error.poke(false.B)
      dut.clock.step(1)

      dut.io.dispatch.expect(false.B)
      dut.io.npu_reset.expect(false.B)
      dut.io.cmd.expect(0.U)
      dut.io.rdata.expect(0.U)  // status = 0b000
    }
  }

  // ── Test 2 ──────────────────────────────────────────────────────────────
  it should "latch base_addr when written" in {
    test(new QuasarCSRFile(xlen)) { dut =>
      dut.io.wen.poke(true.B)
      dut.io.waddr.poke(QuasarCSRAddr.QSR_BASE_ADDR)
      dut.io.wdata.poke(0xdeadbeefL.U)
      dut.io.npu_busy.poke(false.B)
      dut.io.npu_done.poke(false.B)
      dut.io.npu_error.poke(false.B)
      dut.clock.step(1)

      dut.io.base_addr.expect(0xdeadbeefL.U)

      // read it back through CSR read port
      dut.io.raddr.poke(QuasarCSRAddr.QSR_BASE_ADDR)
      dut.io.rdata.expect(0xdeadbeefL.U)
    }
  }

  // ── Test 3 ──────────────────────────────────────────────────────────────
  it should "produce a single-cycle dispatch pulse then self-clear" in {
    test(new QuasarCSRFile(xlen)) { dut =>
      dut.io.npu_busy.poke(false.B)
      dut.io.npu_done.poke(false.B)
      dut.io.npu_error.poke(false.B)

      // write CTRL with bit 0 set
      dut.io.wen.poke(true.B)
      dut.io.waddr.poke(QuasarCSRAddr.QSR_CTRL)
      dut.io.wdata.poke(1.U)
      dut.clock.step(1)

      // dispatch should be high this cycle
      dut.io.dispatch.expect(true.B)

      // de-assert write
      dut.io.wen.poke(false.B)
      dut.clock.step(1)

      // dispatch must have self-cleared
      dut.io.dispatch.expect(false.B)
    }
  }

  // ── Test 4 ──────────────────────────────────────────────────────────────
  it should "reflect npu_busy in status register" in {
    test(new QuasarCSRFile(xlen)) { dut =>
      dut.io.wen.poke(false.B)
      dut.io.waddr.poke(0.U)
      dut.io.wdata.poke(0.U)
      dut.io.npu_busy.poke(true.B)
      dut.io.npu_done.poke(false.B)
      dut.io.npu_error.poke(false.B)
      dut.io.raddr.poke(QuasarCSRAddr.QSR_STATUS)
      dut.clock.step(1)

      // status bit 0 = busy = 1
      dut.io.rdata.expect(1.U)
    }
  }

  // ── Test 5 ──────────────────────────────────────────────────────────────
  it should "decode shape_m and shape_n from packed shape register" in {
    test(new QuasarCSRFile(xlen)) { dut =>
      dut.io.npu_busy.poke(false.B)
      dut.io.npu_done.poke(false.B)
      dut.io.npu_error.poke(false.B)

      // M=16, N=32 packed as {N[15:0], M[15:0]}
      val packed = (32 << 16) | 16
      dut.io.wen.poke(true.B)
      dut.io.waddr.poke(QuasarCSRAddr.QSR_SHAPE)
      dut.io.wdata.poke(packed.U)
      dut.clock.step(1)

      dut.io.shape_m.expect(16.U)
      dut.io.shape_n.expect(32.U)
    }
  }

  // ── Test 6 ──────────────────────────────────────────────────────────────
  it should "correctly identify NPU CSR addresses" in {
    test(new QuasarCSRFile(xlen)) { dut =>
      // We test the companion object logic directly in Scala (not hardware)
      assert(QuasarCSRAddr.isNpuCSR(0x800) == true)
      assert(QuasarCSRAddr.isNpuCSR(0x805) == true)
      assert(QuasarCSRAddr.isNpuCSR(0x300) == false)
      assert(QuasarCSRAddr.isNpuCSR(0x806) == false)
    }
  }
}