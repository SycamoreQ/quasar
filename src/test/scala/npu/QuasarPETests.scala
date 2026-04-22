package npu

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec

/**
 * QuasarPETests: functional verification of a single Processing Element.
 *
 * Pipeline latency reminder:
 *   The MAC inside the PE has 4 register stages.
 *   act_out is delayed by 1 register stage (act_reg).
 *   psum_out is valid when mac.valid_out is high, which is 4 cycles
 *   after valid_in is presented — same timing as QuasarMACTests.
 *
 * Test convention:
 *   loadWeight(dut, w)      — load weight w into the PE, step 1 cycle
 *   driveAct(dut, a, ...)   — drive activation and control signals
 *   flush(dut, n)           — drive zeros for n cycles to drain pipeline
 */
class QuasarPETests extends AnyFlatSpec with ChiselScalatestTester {

  def loadWeight(dut: QuasarPE, w: Int): Unit = {
    dut.io.weight_load.poke(true.B)
    dut.io.weight_data.poke(w.S)
    dut.io.valid_in.poke(false.B)
    dut.io.clear.poke(false.B)
    dut.io.act_in.poke(0.S)
    dut.io.psum_in.poke(0.S)
    dut.clock.step(1)
    dut.io.weight_load.poke(false.B)
  }

  def driveAct(dut: QuasarPE,
               act: Int,
               psum: Int   = 0,
               valid: Boolean = true,
               clear: Boolean = false): Unit = {
    dut.io.act_in.poke(act.S)
    dut.io.psum_in.poke(psum.S)
    dut.io.valid_in.poke(valid.B)
    dut.io.clear.poke(clear.B)
    dut.io.weight_load.poke(false.B)
  }

  def flush(dut: QuasarPE, cycles: Int = 4): Unit = {
    driveAct(dut, 0, 0, valid = false)
    dut.clock.step(cycles)
  }

  behavior.of("QuasarPE")

  // ── Test 1 ────────────────────────────────────────────────────────────
  // After reset, all outputs are zero/false.
  it should "reset to safe defaults" in {
    test(new QuasarPE()) { dut =>
      dut.io.psum_out.expect(0.S)
      dut.io.act_out.expect(0.S)
      dut.io.valid_out.expect(false.B)
      dut.io.overflow.expect(false.B)
    }
  }

  // ── Test 2 ────────────────────────────────────────────────────────────
  // Weight loads correctly and does not affect pipeline outputs.
  it should "load a weight without disturbing pipeline outputs" in {
    test(new QuasarPE()) { dut =>
      loadWeight(dut, 5)
      // After weight load, valid_out must still be false
      dut.io.valid_out.expect(false.B)
      dut.io.psum_out.expect(0.S)
    }
  }

  // ── Test 3 ────────────────────────────────────────────────────────────
  // Single MAC: weight=3, act=4, psum_in=0
  // Expected psum_out = 3*4 + 0 = 12
  it should "compute weight * activation correctly" in {
    test(new QuasarPE()) { dut =>
      loadWeight(dut, 3)
      driveAct(dut, 4, psum = 0, valid = true, clear = true)
      dut.clock.step(1)
      flush(dut, 4)
      dut.io.valid_out.expect(true.B)
      dut.io.psum_out.expect(12.S)
    }
  }

  // ── Test 4 ────────────────────────────────────────────────────────────
  // Partial sum addition: weight=2, act=5, psum_in=100
  // Expected psum_out = 2*5 + 100 = 110
  // This tests the systolic column accumulation behavior.
  it should "add psum_in to MAC result" in {
    test(new QuasarPE()) { dut =>
      loadWeight(dut, 2)
      driveAct(dut, 5, psum = 100, valid = true, clear = true)
      dut.clock.step(1)
      flush(dut, 4)
      dut.io.valid_out.expect(true.B)
      dut.io.psum_out.expect(110.S)
    }
  }

  // ── Test 5 ────────────────────────────────────────────────────────────
  // act_out is a registered copy of act_in — arrives one cycle later.
  // Feed act_in=7, check that act_out=7 after one clock edge.
  it should "pass activation through with one cycle delay" in {
    test(new QuasarPE()) { dut =>
      loadWeight(dut, 1)
      driveAct(dut, 7, valid = false) // valid=false so MAC doesn't run
      dut.clock.step(1)
      dut.io.act_out.expect(7.S)
    }
  }

  // ── Test 6 ────────────────────────────────────────────────────────────
  // act_out reflects the activation from TWO cycles ago after two steps.
  // This verifies act_reg correctly holds the previous value.
  it should "hold act_out stable between activations" in {
    test(new QuasarPE()) { dut =>
      loadWeight(dut, 1)
      driveAct(dut, 7, valid = false)
      dut.clock.step(1)
      dut.io.act_out.expect(7.S)
      // Drive different activation
      driveAct(dut, 3, valid = false)
      dut.clock.step(1)
      dut.io.act_out.expect(3.S)
    }
  }

  // ── Test 7 ────────────────────────────────────────────────────────────
  // Dot product of length 3 with psum accumulation from above.
  // weight=2, activations=[1,3,5], psum_in=10 on last cycle
  // MAC accumulates: 2*1 + 2*3 + 2*5 = 2 + 6 + 10 = 18
  // psum_out = 18 + 10 = 28
  // Note: psum_in=10 is only added at output stage when valid_out
  // is high — feed psum_in=10 throughout but it only matters then.
  it should "accumulate a dot product and add psum_in" in {
    test(new QuasarPE()) { dut =>
      loadWeight(dut, 2)

      driveAct(dut, 1, psum = 10, valid = true, clear = true)
      dut.clock.step(1)
      driveAct(dut, 3, psum = 10, valid = true, clear = false)
      dut.clock.step(1)
      driveAct(dut, 5, psum = 10, valid = true, clear = false)
      dut.clock.step(1)

      flush(dut, 4)
      dut.io.valid_out.expect(true.B)
      dut.io.psum_out.expect(28.S)
    }
  }

  // ── Test 8 ────────────────────────────────────────────────────────────
  // Weight reload mid-computation: load weight=3, run one activation,
  // then reload weight=5, run another activation.
  // First result: 3*4 = 12 (with clear)
  // Second result: 5*6 = 30 (with clear)
  // Results emerge 4 cycles after each input.
  it should "accept a new weight load between computations" in {
    test(new QuasarPE()) { dut =>
      loadWeight(dut, 3)
      driveAct(dut, 4, psum = 0, valid = true, clear = true)
      dut.clock.step(1)
      flush(dut, 4)
      dut.io.valid_out.expect(true.B)
      dut.io.psum_out.expect(12.S)

      // Reload weight and run second computation
      loadWeight(dut, 5)
      driveAct(dut, 6, psum = 0, valid = true, clear = true)
      dut.clock.step(1)
      flush(dut, 4)
      dut.io.valid_out.expect(true.B)
      dut.io.psum_out.expect(30.S)
    }
  }

  // ── Test 9 ────────────────────────────────────────────────────────────
  // Negative weight: weight=-3, act=4, psum_in=0
  // Expected: -3*4 = -12
  it should "handle negative weights correctly" in {
    test(new QuasarPE()) { dut =>
      loadWeight(dut, -3)
      driveAct(dut, 4, psum = 0, valid = true, clear = true)
      dut.clock.step(1)
      flush(dut, 4)
      dut.io.valid_out.expect(true.B)
      dut.io.psum_out.expect(-12.S)
    }
  }

  // ── Test 10 ───────────────────────────────────────────────────────────
  // psum_in passes through unchanged when valid_out is low.
  // Feed psum_in=42 with valid_in=false — psum_out should be 42
  // because the PE is not producing a result, just passing through.
  it should "pass psum_in through when not producing a result" in {
    test(new QuasarPE()) { dut =>
      loadWeight(dut, 1)
      driveAct(dut, 0, psum = 42, valid = false)
      dut.clock.step(1)
      dut.io.valid_out.expect(false.B)
      dut.io.psum_out.expect(42.S)
    }
  }
}