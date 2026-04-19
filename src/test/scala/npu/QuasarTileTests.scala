package npu

import chisel3._
import chiseltest._
import mini._
import org.scalatest.flatspec.AnyFlatSpec

class QuasarTileTests extends AnyFlatSpec with ChiselScalatestTester {

  def makeConfig() = MiniConfig()

  behavior.of("QuasarTile")

  it should "elaborate without errors" in {
    val cfg = makeConfig()
    test(new QuasarTile(cfg.core, cfg.nasti, cfg.cache)) { dut =>
      dut.clock.step(1)
      // If we get here, elaboration and reset succeeded
      succeed
    }
  }

  it should "expose npu_status = 0b010 (done=1) at reset with stub NPU" in {
    val cfg = makeConfig()
    test(new QuasarTile(cfg.core, cfg.nasti, cfg.cache)) { dut =>
      dut.clock.step(2)
      // stub NPU: busy=0, done=1, error=0 → status = 0b010
      dut.io.npu_status.expect(2.U)
    }
  }
}