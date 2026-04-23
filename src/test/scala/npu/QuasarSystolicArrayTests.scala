package npu

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec

/**
 * QuasarSystolicArrayTests: functional verification of the 4x4 array.
 *
 * Pipeline latency in the array:
 *   Each PE has 4 MAC pipeline stages.
 *   Activation skewing adds 0/1/2/3 cycles for rows 0/1/2/3.
 *   The last row (row 3) sees its first activation 3 cycles after
 *   row 0, and its result emerges 4 cycles after that = 7 total
 *   cycles from when row 0's first input is presented.
 *
 *   BUT: we are NOT implementing the skew buffer inside the array.
 *   The IO accepts pre-skewed activations. The test driver handles
 *   skewing by feeding rows with appropriate delays.
 *
 * Test convention:
 *   loadWeights(dut, w)    — broadcast weight matrix w to all PEs
 *   driveRow(dut, acts, ..)— drive one activation per row this cycle
 *   drainAndCheck(dut, ...) — flush pipeline and verify outputs
 */
class QuasarSystolicArrayTests extends AnyFlatSpec with ChiselScalatestTester {

  val p = QuasarArrayParams(rows = 4, cols = 4)

  // Load a 4x4 weight matrix into the array
  def loadWeights(dut: QuasarSystolicArray,
                  weights: Seq[Seq[Int]]): Unit = {
    dut.io.weights.weight_load.poke(true.B)
    for (i <- 0 until p.rows) {
      for (j <- 0 until p.cols) {
        dut.io.weights.weight_data(i)(j).poke(weights(i)(j).S)
      }
    }
    dut.io.valid_in.poke(false.B)
    dut.io.clear.poke(false.B)
    for (i <- 0 until p.rows) dut.io.act_in(i).poke(0.S)
    dut.clock.step(1)
    dut.io.weights.weight_load.poke(false.B)
  }

  // Drive one activation value per row, all rows simultaneously
  def driveActs(dut: QuasarSystolicArray,
                acts: Seq[Int],
                valid: Boolean = true,
                clear: Boolean = false): Unit = {
    for (i <- 0 until p.rows) dut.io.act_in(i).poke(acts(i).S)
    dut.io.valid_in.poke(valid.B)
    dut.io.clear.poke(clear.B)
    dut.io.weights.weight_load.poke(false.B)
  }

  // Flush pipeline with no-op inputs for n cycles
  def flush(dut: QuasarSystolicArray, cycles: Int = 3): Unit = {
    driveActs(dut, Seq.fill(p.rows)(0), valid = false)
    dut.clock.step(cycles)
  }

  behavior.of("QuasarSystolicArray")

  // ── Test 1 ────────────────────────────────────────────────────────────
  // Elaboration and reset: all outputs zero, valid_out false.
  it should "reset to safe defaults" in {
    test(new QuasarSystolicArray(p)) { dut =>
      dut.io.valid_out.expect(false.B)
      for (j <- 0 until p.cols) dut.io.psum_out(j).expect(0.S)
    }
  }

  // ── Test 2 ────────────────────────────────────────────────────────────
  // Weight broadcast: load all weights to the same value (2),
  // verify it doesn't disturb valid_out.
  it should "load weights without disturbing outputs" in {
    test(new QuasarSystolicArray(p)) { dut =>
      loadWeights(dut, Seq.fill(p.rows)(Seq.fill(p.cols)(2)))
      dut.io.valid_out.expect(false.B)
    }
  }

  // ── Test 3 ────────────────────────────────────────────────────────────
  // Identity weight matrix: all diagonal weights=1, off-diagonal=0.
  // Feed activation vector [1,2,3,4] to all rows simultaneously.
  // Each PE[i][j]: weight= if i==j then 1 else 0
  // Column j output = sum over rows i of (weight[i][j] * act[i])
  //                 = 1 * act[j]  (only diagonal contributes)
  // Expected psum_out = [1, 2, 3, 4]
  it should "compute identity matrix multiply correctly" in {
    test(new QuasarSystolicArray(p)) { dut =>
      val identityWeights = Seq.tabulate(p.rows, p.cols) { (i, j) =>
        if (i == j) 1 else 0
      }
      loadWeights(dut, identityWeights)

      driveActs(dut, Seq(1, 2, 3, 4), valid = true, clear = true)
      dut.clock.step(1)
      flush(dut, 4)
      dut.clock.step(1)
      dut.clock.step(1)
      dut.clock.step(1)
      dut.io.valid_out.expect(true.B)
      dut.io.psum_out(0).expect(1.S)
      dut.io.psum_out(1).expect(2.S)
      dut.io.psum_out(2).expect(3.S)
      dut.io.psum_out(3).expect(4.S)
    }
  }

  // ── Test 4 ────────────────────────────────────────────────────────────
  // Uniform weight matrix: all weights = 1.
  // Feed activation vector [1, 1, 1, 1].
  // Each column receives one activation per row, all=1, weight=1.
  // Each PE contributes 1*1=1. Column sum = 4 (one per row).
  // Expected psum_out = [4, 4, 4, 4]
  it should "accumulate across all rows correctly" in {
    test(new QuasarSystolicArray(p)) { dut =>
      loadWeights(dut, Seq.fill(p.rows)(Seq.fill(p.cols)(1)))

      driveActs(dut, Seq(1, 1, 1, 1), valid = true, clear = true)
      dut.clock.step(1)
      flush(dut, 4)

      dut.io.valid_out.expect(true.B)
      for (j <- 0 until p.cols) dut.io.psum_out(j).expect(4.S)
    }
  }

  // ── Test 5 ────────────────────────────────────────────────────────────
  // Scaled activation: weight=2 everywhere, act=[3,3,3,3]
  // Each PE contributes 2*3=6. Four rows accumulate: 4*6=24.
  // Expected psum_out = [24, 24, 24, 24]
  it should "scale activations by weights correctly" in {
    test(new QuasarSystolicArray(p)) { dut =>
      loadWeights(dut, Seq.fill(p.rows)(Seq.fill(p.cols)(2)))

      driveActs(dut, Seq(3, 3, 3, 3), valid = true, clear = true)
      dut.clock.step(1)
      flush(dut, 4)

      dut.io.valid_out.expect(true.B)
      for (j <- 0 until p.cols) dut.io.psum_out(j).expect(24.S)
    }
  }

  // ── Test 6 ────────────────────────────────────────────────────────────
  // Column isolation: weight matrix has only column 2 non-zero (all 3s).
  // Feed act=[1,1,1,1]. Only psum_out(2) should be non-zero.
  // psum_out(2) = 4 * (3*1) = 12
  it should "isolate computation to specific columns" in {
    test(new QuasarSystolicArray(p)) { dut =>
      val colWeights = Seq.tabulate(p.rows, p.cols) { (i, j) =>
        if (j == 2) 3 else 0
      }
      loadWeights(dut, colWeights)

      driveActs(dut, Seq(1, 1, 1, 1), valid = true, clear = true)
      dut.clock.step(1)
      flush(dut, 4)

      dut.io.valid_out.expect(true.B)
      dut.io.psum_out(0).expect(0.S)
      dut.io.psum_out(1).expect(0.S)
      dut.io.psum_out(2).expect(12.S)
      dut.io.psum_out(3).expect(0.S)
    }
  }

  // ── Test 7 ────────────────────────────────────────────────────────────
  // Multi-cycle dot product (K=2):
  // Weights all=1. Feed [1,1,1,1] then [2,2,2,2] with clear on first.
  // Each column accumulates: (1*1 + 1*2) * 4 rows = 3 * 4 = 12
  // Expected psum_out = [12, 12, 12, 12]
  it should "accumulate multi-cycle dot products" in {
    test(new QuasarSystolicArray(p)) { dut =>
      loadWeights(dut, Seq.fill(p.rows)(Seq.fill(p.cols)(1)))

      driveActs(dut, Seq(1, 1, 1, 1), valid = true, clear = true)
      dut.clock.step(1)
      driveActs(dut, Seq(2, 2, 2, 2), valid = true, clear = false)
      dut.clock.step(1)
      flush(dut, 23)

      dut.io.valid_out.expect(true.B)
      for (j <- 0 until p.cols) dut.io.psum_out(j).expect(12.S)
    }
  }

  // ── Test 8 ────────────────────────────────────────────────────────────
  // Negative weights: weight=-1 everywhere, act=[2,2,2,2]
  // Each PE: -1*2=-2. Four rows: 4*(-2)=-8.
  // Expected psum_out = [-8, -8, -8, -8]
  it should "handle negative weights across the array" in {
    test(new QuasarSystolicArray(p)) { dut =>
      loadWeights(dut, Seq.fill(p.rows)(Seq.fill(p.cols)(-1)))

      driveActs(dut, Seq(2, 2, 2, 2), valid = true, clear = true)
      dut.clock.step(1)
      flush(dut, 4)

      dut.io.valid_out.expect(true.B)
      for (j <- 0 until p.cols) dut.io.psum_out(j).expect(-8.S)
    }
  }
}