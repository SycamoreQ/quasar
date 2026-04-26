package npu

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec

class QuasarSystolicArrayTests extends AnyFlatSpec with ChiselScalatestTester {

  val p = QuasarArrayParams(rows = 4, cols = 4)

  def loadWeights(dut: QuasarSystolicArray,
                  weights: Seq[Seq[Int]]): Unit = {
    dut.io.weights.weight_load.poke(true.B)
    for (i <- 0 until p.rows)
      for (j <- 0 until p.cols)
        dut.io.weights.weight_data(i)(j).poke(weights(i)(j).S)
    dut.io.valid_in.poke(false.B)
    dut.io.clear.poke(false.B)
    for (i <- 0 until p.rows) dut.io.act_in(i).poke(0.S)
    dut.clock.step(1)
    dut.io.weights.weight_load.poke(false.B)
  }

  def driveActs(dut: QuasarSystolicArray,
                acts: Seq[Int],
                valid: Boolean = true,
                clear: Boolean = false): Unit = {
    for (i <- 0 until p.rows) dut.io.act_in(i).poke(acts(i).S)
    dut.io.valid_in.poke(valid.B)
    dut.io.clear.poke(clear.B)
    dut.io.weights.weight_load.poke(false.B)
  }

  // Step n cycles with no-op inputs, do not check inside
  def drain(dut: QuasarSystolicArray, cycles: Int): Unit = {
    driveActs(dut, Seq.fill(p.rows)(0), valid = false)
    dut.clock.step(cycles)
  }

  // Read all column outputs at their individual valid cycles.
  // Column j fires at baseDelay + j cycles after the last input step.
  // This method steps one cycle at a time, collecting each column's
  // result as it becomes valid.
  // baseDelay = 3 (from debug: col 0 fires 3 cycles after input step)
  def readAllColumns(dut: QuasarSystolicArray,
                     baseDelay: Int = 3): Seq[Long] = {
    driveActs(dut, Seq.fill(p.rows)(0), valid = false)
    val results = Array.fill(p.cols)(0L)
    // Step baseDelay + cols - 1 cycles total, collecting each column
    for (cycle <- 1 to baseDelay + p.cols - 1) {
      dut.clock.step(1)
      for (j <- 0 until p.cols) {
        if (cycle == baseDelay + j) {
          // Column j is valid on this cycle
          assert(dut.io.valid_out(j).peek().litToBoolean,
            s"Column $j valid_out not high at cycle $cycle")
          results(j) = dut.io.psum_out(j).peek().litValue.toLong
        }
      }
    }
    results.toSeq
  }

  behavior.of("QuasarSystolicArray")

  it should "reset to safe defaults" in {
    test(new QuasarSystolicArray(p)) { dut =>
      for (j <- 0 until p.cols) dut.io.valid_out(j).expect(false.B)
      for (j <- 0 until p.cols) dut.io.psum_out(j).expect(0.S)
    }
  }

  it should "load weights without disturbing outputs" in {
    test(new QuasarSystolicArray(p)) { dut =>
      loadWeights(dut, Seq.fill(p.rows)(Seq.fill(p.cols)(2)))
      for (j <- 0 until p.cols) dut.io.valid_out(j).expect(false.B)
    }
  }

  it should "compute identity matrix multiply correctly" in {
    test(new QuasarSystolicArray(p)) { dut =>
      val identityWeights = Seq.tabulate(p.rows, p.cols)((i,j) =>
        if (i == j) 1 else 0)
      loadWeights(dut, identityWeights)
      driveActs(dut, Seq(1,2,3,4), valid=true, clear=true)
      dut.clock.step(1)
      val results = readAllColumns(dut)
      assert(results(0) == 1, s"col0=${results(0)}")
      assert(results(1) == 2, s"col1=${results(1)}")
      assert(results(2) == 3, s"col2=${results(2)}")
      assert(results(3) == 4, s"col3=${results(3)}")
    }
  }

  it should "accumulate across all rows correctly" in {
    test(new QuasarSystolicArray(p)) { dut =>
      loadWeights(dut, Seq.fill(p.rows)(Seq.fill(p.cols)(1)))
      driveActs(dut, Seq(1,1,1,1), valid=true, clear=true)
      dut.clock.step(1)
      val results = readAllColumns(dut)
      for (j <- 0 until p.cols)
        assert(results(j) == 4, s"col$j=${results(j)}")
    }
  }

  it should "scale activations by weights correctly" in {
    test(new QuasarSystolicArray(p)) { dut =>
      loadWeights(dut, Seq.fill(p.rows)(Seq.fill(p.cols)(2)))
      driveActs(dut, Seq(3,3,3,3), valid=true, clear=true)
      dut.clock.step(1)
      val results = readAllColumns(dut)
      for (j <- 0 until p.cols)
        assert(results(j) == 24, s"col$j=${results(j)}")
    }
  }

  it should "isolate computation to specific columns" in {
    test(new QuasarSystolicArray(p)) { dut =>
      val colWeights = Seq.tabulate(p.rows, p.cols)((i,j) =>
        if (j == 2) 3 else 0)
      loadWeights(dut, colWeights)
      driveActs(dut, Seq(1,1,1,1), valid=true, clear=true)
      dut.clock.step(1)
      val results = readAllColumns(dut)
      assert(results(0) == 0,  s"col0=${results(0)}")
      assert(results(1) == 0,  s"col1=${results(1)}")
      assert(results(2) == 12, s"col2=${results(2)}")
      assert(results(3) == 0,  s"col3=${results(3)}")
    }
  }

  it should "accumulate multi-cycle dot products" in {
    test(new QuasarSystolicArray(p)) { dut =>
      loadWeights(dut, Seq.fill(p.rows)(Seq.fill(p.cols)(1)))
      driveActs(dut, Seq(1,1,1,1), valid=true, clear=true)
      dut.clock.step(1)
      driveActs(dut, Seq(2,2,2,2), valid=true, clear=false)
      dut.clock.step(1)
      // Two inputs: base delay is still 3 from last input step
      val results = readAllColumns(dut)
      for (j <- 0 until p.cols)
        assert(results(j) == 12, s"col$j=${results(j)}")
    }
  }

  it should "handle negative weights across the array" in {
    test(new QuasarSystolicArray(p)) { dut =>
      loadWeights(dut, Seq.fill(p.rows)(Seq.fill(p.cols)(-1)))
      driveActs(dut, Seq(2,2,2,2), valid=true, clear=true)
      dut.clock.step(1)
      val results = readAllColumns(dut)
      for (j <- 0 until p.cols)
        assert(results(j) == -8, s"col$j=${results(j)}")
    }
  }
}