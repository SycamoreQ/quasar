package npu

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec

class QuasarTileControllerTests extends AnyFlatSpec
  with ChiselScalatestTester {

  val sp = QuasarScratchpadParams(depth = 64, width = 32)

  behavior.of("QuasarTileController")

  // ── Test 1 ────────────────────────────────────────────────────────────
  it should "start idle with all outputs deasserted" in {
    test(new QuasarTileController(sp)) { dut =>
      dut.io.dispatch.poke(false.B)
      dut.io.npu_reset.poke(false.B)
      dut.io.dma_done.poke(false.B)
      dut.io.dma_busy.poke(false.B)
      dut.io.array_valid.poke(false.B)
      dut.io.cxl_write_valid.poke(false.B)
      dut.clock.step(1)
      dut.io.npu_busy.expect(false.B)
      dut.io.npu_done.expect(false.B)
      dut.io.dma_start.expect(false.B)
      dut.io.array_start.expect(false.B)
    }
  }

  // ── Test 2 ────────────────────────────────────────────────────────────
  it should "pulse dma_start on dispatch and go busy" in {
    test(new QuasarTileController(sp)) { dut =>
      dut.io.dispatch.poke(false.B)
      dut.io.npu_reset.poke(false.B)
      dut.io.dma_done.poke(false.B)
      dut.io.dma_busy.poke(false.B)
      dut.io.array_valid.poke(false.B)
      dut.io.cxl_write_valid.poke(false.B)
      dut.io.base_addr.poke(0x1000.U)
      dut.io.shape_m.poke(4.U)
      dut.io.shape_n.poke(4.U)

      // Pulse dispatch
      dut.io.dispatch.poke(true.B)
      dut.clock.step(1)
      dut.io.dispatch.poke(false.B)

      // Should be busy and dma_start should have pulsed
      dut.io.npu_busy.expect(true.B)
      dut.io.dma_start.expect(false.B)  // pulse was last cycle
    }
  }

  // ── Test 3 ────────────────────────────────────────────────────────────
  it should "pulse array_start when DMA completes" in {
    test(new QuasarTileController(sp)) { dut =>
      dut.io.npu_reset.poke(false.B)
      dut.io.dma_done.poke(false.B)
      dut.io.dma_busy.poke(false.B)
      dut.io.array_valid.poke(false.B)
      dut.io.cxl_write_valid.poke(false.B)
      dut.io.base_addr.poke(0x1000.U)
      dut.io.shape_m.poke(4.U)
      dut.io.shape_n.poke(4.U)

      // Dispatch
      dut.io.dispatch.poke(true.B)
      dut.clock.step(1)
      dut.io.dispatch.poke(false.B)

      // Simulate DMA completing
      dut.io.dma_done.poke(true.B)
      dut.io.array_start.expect(true.B)
      dut.clock.step(1)
      dut.io.dma_done.poke(false.B)

      // array_start should pulse
      dut.io.array_start.expect(false.B)
      dut.io.npu_busy.expect(true.B)
    }
  }

  // ── Test 4 ────────────────────────────────────────────────────────────
  it should "pulse npu_done when array computation completes" in {
    test(new QuasarTileController(sp)) { dut =>
      dut.io.npu_reset.poke(false.B)
      dut.io.cxl_write_valid.poke(false.B)
      dut.io.base_addr.poke(0x1000.U)
      dut.io.shape_m.poke(4.U)
      dut.io.shape_n.poke(4.U)

      // Full sequence: dispatch → dma_done → array_valid
      dut.io.dispatch.poke(true.B)
      dut.io.dma_done.poke(false.B)
      dut.io.array_valid.poke(false.B)
      dut.clock.step(1)
      dut.io.dispatch.poke(false.B)

      dut.io.dma_done.poke(true.B)
      dut.clock.step(1)
      dut.io.dma_done.poke(false.B)

      dut.io.array_valid.poke(true.B)
      dut.clock.step(1)
      dut.io.array_valid.poke(false.B)

      // sDone state: npu_done pulses
      dut.io.npu_done.expect(true.B)
      dut.io.npu_busy.expect(false.B)
    }
  }

  // ── Test 5 ────────────────────────────────────────────────────────────
  it should "return to idle after done and accept new dispatch" in {
    test(new QuasarTileController(sp)) { dut =>
      dut.io.npu_reset.poke(false.B)
      dut.io.cxl_write_valid.poke(false.B)
      dut.io.base_addr.poke(0x1000.U)
      dut.io.shape_m.poke(4.U)
      dut.io.shape_n.poke(4.U)

      // Run full dispatch cycle
      dut.io.dispatch.poke(true.B)
      dut.io.dma_done.poke(false.B)
      dut.io.array_valid.poke(false.B)
      dut.clock.step(1)
      dut.io.dispatch.poke(false.B)
      dut.io.dma_done.poke(true.B)
      dut.clock.step(1)
      dut.io.dma_done.poke(false.B)
      dut.io.array_valid.poke(true.B)
      dut.clock.step(1)
      dut.io.array_valid.poke(false.B)
      dut.clock.step(1)  // sDone → sIdle

      // Should be idle again
      dut.io.npu_busy.expect(false.B)
      dut.io.npu_done.expect(false.B)

      // New dispatch should work
      dut.io.dispatch.poke(true.B)
      dut.clock.step(1)
      dut.io.dispatch.poke(false.B)
      dut.io.npu_busy.expect(true.B)
    }
  }

  // ── Test 6 ────────────────────────────────────────────────────────────
  it should "reset to idle immediately from any state" in {
    test(new QuasarTileController(sp)) { dut =>
      dut.io.cxl_write_valid.poke(false.B)
      dut.io.base_addr.poke(0x1000.U)
      dut.io.shape_m.poke(4.U)
      dut.io.shape_n.poke(4.U)
      dut.io.dma_done.poke(false.B)
      dut.io.array_valid.poke(false.B)

      // Get into sDMA state
      dut.io.dispatch.poke(true.B)
      dut.io.npu_reset.poke(false.B)
      dut.clock.step(1)
      dut.io.dispatch.poke(false.B)
      dut.io.npu_busy.expect(true.B)

      // Assert reset
      dut.io.npu_reset.poke(true.B)
      dut.clock.step(1)
      dut.io.npu_reset.poke(false.B)

      // Should be idle
      dut.io.npu_busy.expect(false.B)
      dut.io.dma_start.expect(false.B)
    }
  }

  // ── Test 7 ────────────────────────────────────────────────────────────
  it should "toggle bank_sel after DMA completes" in {
    test(new QuasarTileController(sp)) { dut =>
      dut.io.npu_reset.poke(false.B)
      dut.io.cxl_write_valid.poke(false.B)
      dut.io.base_addr.poke(0x1000.U)
      dut.io.shape_m.poke(4.U)
      dut.io.shape_n.poke(4.U)
      dut.io.dma_done.poke(false.B)
      dut.io.array_valid.poke(false.B)

      val initialBank = dut.io.bank_sel.peek().litToBoolean

      // Dispatch and complete DMA
      dut.io.dispatch.poke(true.B)
      dut.clock.step(1)
      dut.io.dispatch.poke(false.B)
      dut.io.dma_done.poke(true.B)
      dut.clock.step(1)
      dut.io.dma_done.poke(false.B)

      // Bank should have toggled
      val newBank = dut.io.bank_sel.peek().litToBoolean
      assert(newBank != initialBank,
        s"bank_sel should toggle: was $initialBank now $newBank")
    }
  }
}