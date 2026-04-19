package npu

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec

/**
 * QuasarMACTests: functional verification of the 3-stage MAC pipeline.
 *
 * Test philosophy: we treat the MAC as a black box and verify
 * observable behavior at the IO boundary. We account for the 3-cycle
 * pipeline latency explicitly in every test — inputs fed on cycle N
 * produce outputs on cycle N+3 (after 3 rising edges).
 *
 * Helper: stepAndRead
 *   Drives inputs for one cycle, steps the clock, returns nothing.
 *   The caller is responsible for reading outputs at the right offset.
 */
class QuasarMACTests extends AnyFlatSpec with ChiselScalatestTester {

  // Drive inputs for one cycle (does not step clock)
  def drive(dut: QuasarMAC,
            a: Int, b: Int,
            valid: Boolean = true,
            clear: Boolean = false): Unit = {
    dut.io.a_in.poke(a.S)
    dut.io.b_in.poke(b.S)
    dut.io.valid_in.poke(valid.B)
    dut.io.clear.poke(clear.B)
  }

  // Flush the pipeline with no-ops (valid=false)
  def flush(dut: QuasarMAC, cycles: Int = 3): Unit = {
    drive(dut, 0, 0, valid = false)
    dut.clock.step(cycles)
  }

  behavior.of("QuasarMAC")

  // ── Test 1 ────────────────────────────────────────────────────────────
  // After reset, outputs must be zero and valid must be low.
  it should "reset to safe defaults" in {
    test(new QuasarMAC()) { dut =>
      dut.io.result_out.expect(0.S)
      dut.io.valid_out.expect(false.B)
      dut.io.overflow.expect(false.B)
    }
  }

  // ── Test 2 ────────────────────────────────────────────────────────────
  // Single multiply: feed one valid input pair, flush 3 cycles,
  // verify the product appears at the output.
  // 3 * 4 = 12, with clear=true so accumulator = 12
  it should "compute a single multiply-accumulate correctly" in {
    test(new QuasarMAC()) { dut =>
      drive(dut, 3, 4, valid = true, clear = true)
      dut.clock.step(1)
      flush(dut, 3)
      dut.io.valid_out.expect(true.B)
      dut.io.result_out.expect(12.S)
    }
  }

  // ── Test 3 ────────────────────────────────────────────────────────────
  // Dot product: 3 accumulations without clear in between.
  // [2,3,4] · [5,6,7] = 10 + 18 + 28 = 56
  it should "accumulate a dot product of length 3" in {
    test(new QuasarMAC()) { dut =>
      val as = Seq(2, 3, 4)
      val bs = Seq(5, 6, 7)

      // Feed first pair with clear to reset accumulator
      drive(dut, as(0), bs(0), valid = true, clear = true)
      dut.clock.step(1)

      // Feed remaining pairs without clear
      for (i <- 1 until as.length) {
        drive(dut, as(i), bs(i), valid = true, clear = false)
        dut.clock.step(1)
      }

      // Flush pipeline
      flush(dut, 3)

      dut.io.valid_out.expect(true.B)
      dut.io.result_out.expect(56.S)
    }
  }

  // ── Test 4 ────────────────────────────────────────────────────────────
  // Clear between two dot products.
  // First: [1,2] · [3,4] = 3 + 8 = 11
  // Then clear, then: [5,6] · [7,8] = 35 + 48 = 83
  // Final output must be 83, not 11+83=94
  it should "reset accumulator between two dot products via clear" in {
    test(new QuasarMAC()) { dut =>
      // First dot product
      drive(dut, 1, 3, valid = true, clear = true)
      dut.clock.step(1)
      drive(dut, 2, 4, valid = true, clear = false)
      dut.clock.step(1)

      // Second dot product starts with clear
      drive(dut, 5, 7, valid = true, clear = true)
      dut.clock.step(1)
      drive(dut, 6, 8, valid = true, clear = false)
      dut.clock.step(1)

      flush(dut, 3)
      dut.io.valid_out.expect(true.B)
      dut.io.result_out.expect(83.S)
    }
  }

  // ── Test 5 ────────────────────────────────────────────────────────────
  // Negative operands: (-3) * 4 = -12, accumulated with clear
  it should "handle negative operands correctly" in {
    test(new QuasarMAC()) { dut =>
      drive(dut, -3, 4, valid = true, clear = true)
      dut.clock.step(1)
      flush(dut, 3)
      dut.io.valid_out.expect(true.B)
      dut.io.result_out.expect(-12.S)
    }
  }

  // ── Test 6 ────────────────────────────────────────────────────────────
  // Both negative: (-5) * (-6) = 30
  it should "handle both negative operands" in {
    test(new QuasarMAC()) { dut =>
      drive(dut, -5, -6, valid = true, clear = true)
      dut.clock.step(1)
      flush(dut, 3)
      dut.io.valid_out.expect(true.B)
      dut.io.result_out.expect(30.S)
    }
  }

  // ── Test 7 ────────────────────────────────────────────────────────────
  // valid_out must be low for the first 3 cycles after a valid input
  // (pipeline fill latency)
  it should "hold valid_out low during pipeline fill" in {
    test(new QuasarMAC()) { dut =>
      drive(dut, 1, 1, valid = true, clear = true)
      // Cycle 1: input presented
      dut.clock.step(1)
      dut.io.valid_out.expect(false.B)
      // Cycle 2
      drive(dut, 0, 0, valid = false)
      dut.clock.step(1)
      dut.io.valid_out.expect(false.B)
      // Cycle 3
      dut.clock.step(1)
      dut.io.valid_out.expect(false.B)
      // Cycle 4: output should now be valid
      dut.clock.step(1)
      dut.io.valid_out.expect(true.B)
    }
  }

  // ── Test 8 ────────────────────────────────────────────────────────────
  // Max INT8 values: 127 * 127 = 16129
  // Accumulated 4 times without clear: 4 * 16129 = 64516
  // Must fit in INT32 with no overflow
  it should "accumulate max INT8 products without overflow" in {
    test(new QuasarMAC()) { dut =>
      drive(dut, 127, 127, valid = true, clear = true)
      dut.clock.step(1)
      for (_ <- 0 until 3) {
        drive(dut, 127, 127, valid = true, clear = false)
        dut.clock.step(1)
      }
      flush(dut, 3)
      dut.io.overflow.expect(false.B)
      dut.io.result_out.expect(64516.S)
    }
  }

  // ── Test 9 ────────────────────────────────────────────────────────────
  // Pipeline throughput: feed 5 consecutive valid pairs,
  // verify outputs emerge one per cycle after the initial 3-cycle latency.
  // Pairs: (1,1), (2,2), (3,3), (4,4), (5,5) each with clear=true
  // Expected results (each is its own product since clear resets): 1,4,9,16,25
  it should "sustain one output per cycle after pipeline fill" in {
    test(new QuasarMAC()) { dut =>
      val inputs   = Seq((1,1),(2,2),(3,3),(4,4),(5,5))
      val expected = Seq(1, 4, 9, 16, 25)

      // Feed first 4 inputs to fill the pipeline
      // No reads yet — pipeline not full
      for ((a,b) <- inputs.take(4)) {
        drive(dut, a, b, valid = true, clear = true)
        dut.clock.step(1)
      }

      // From here: result of input 1 is now in result_out_reg
      // Feed remaining inputs AND read outputs simultaneously
      // This is the true streaming mode: one in, one out per cycle
      drive(dut, inputs(4)._1, inputs(4)._2, valid = true, clear = true)
      dut.io.valid_out.expect(true.B)
      dut.io.result_out.expect(1.S)
      dut.clock.step(1)

      // Now drain remaining results with no-op inputs
      drive(dut, 0, 0, valid = false)
      for (exp <- expected.drop(1)) {
        dut.io.valid_out.expect(true.B)
        dut.io.result_out.expect(exp.S)
        dut.clock.step(1)
      }
    }
  }
}