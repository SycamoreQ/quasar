package npu

import chisel3._
import chisel3.util._
import chiseltest._
import junctions._
import org.scalatest.flatspec.AnyFlatSpec

class QuasarDMATests extends AnyFlatSpec with ChiselScalatestTester {

  val nastiP = NastiBundleParameters(addrBits = 32, dataBits = 64, idBits = 5)
  val spP    = QuasarScratchpadParams(depth = 64, width = 32)
  val dmaP   = QuasarDMAParams()

  // Serve a sequence of AXI read beats in response to a burst request
  def serveAXIRead(dut: QuasarDMA, data: Seq[Long]): Unit = {
    // Wait for address request
    var timeout = 0
    while (!dut.io.axi.ar.valid.peek().litToBoolean && timeout < 50) {
      dut.clock.step(1)
      timeout += 1
    }
    // Accept the address
    dut.io.axi.ar.ready.poke(true.B)
    dut.clock.step(1)
    dut.io.axi.ar.ready.poke(false.B)

    // Send data beats one at a time
    for ((beat, idx) <- data.zipWithIndex) {
      val isLast = idx == data.size - 1
      dut.io.axi.r.valid.poke(true.B)
      dut.io.axi.r.bits.data.poke(beat.U)
      dut.io.axi.r.bits.id.poke(0.U)
      dut.io.axi.r.bits.resp.poke(0.U)
      dut.io.axi.r.bits.last.poke(isLast.B)

      // Wait for DMA to assert r.ready
      var beatTimeout = 0
      while (!dut.io.axi.r.ready.peek().litToBoolean && beatTimeout < 20) {
        dut.clock.step(1)
        beatTimeout += 1
      }
      dut.clock.step(1)
    }
    dut.io.axi.r.valid.poke(false.B)
    dut.io.axi.r.bits.last.poke(false.B)
  }

  // Wait for done signal with timeout
  def waitForDone(dut: QuasarDMA, maxCycles: Int = 100): Unit = {
    var t = 0
    while (!dut.io.done.peek().litToBoolean && t < maxCycles) {
      dut.clock.step(1)
      t += 1
    }
    assert(t < maxCycles, s"DMA did not complete within $maxCycles cycles")
  }

  // Default tie-offs for AXI slave signals not driven by test
  def tieOffAXISlave(dut: QuasarDMA): Unit = {
    dut.io.axi.ar.ready.poke(false.B)
    dut.io.axi.r.valid.poke(false.B)
    dut.io.axi.r.bits.data.poke(0.U)
    dut.io.axi.r.bits.id.poke(0.U)
    dut.io.axi.r.bits.resp.poke(0.U)
    dut.io.axi.r.bits.last.poke(false.B)
  }

  behavior.of("QuasarDMA")

  // ── Test 1 ────────────────────────────────────────────────────────────
  it should "start idle and not busy" in {
    test(new QuasarDMA(dmaP, spP, nastiP)) { dut =>
      tieOffAXISlave(dut)
      dut.io.start.poke(false.B)
      dut.clock.step(1)
      dut.io.busy.expect(false.B)
      dut.io.done.expect(false.B)
    }
  }

  // ── Test 2 ────────────────────────────────────────────────────────────
  it should "go busy on start pulse" in {
    test(new QuasarDMA(dmaP, spP, nastiP)) { dut =>
      tieOffAXISlave(dut)
      dut.io.src_addr.poke(0x1000.U)
      dut.io.dst_addr.poke(0.U)
      dut.io.length.poke(4.U)
      dut.io.start.poke(true.B)
      dut.clock.step(1)
      dut.io.start.poke(false.B)
      // Should be busy immediately after start
      dut.io.busy.expect(true.B)
      // Clean up — serve the AXI read so DMA can complete
      fork {
        serveAXIRead(dut, Seq(0x11L, 0x22L, 0x33L, 0x44L))
      }.join()
      waitForDone(dut)
    }
  }

  // ── Test 3 ────────────────────────────────────────────────────────────
  it should "assert done when transfer completes" in {
    test(new QuasarDMA(dmaP, spP, nastiP)) { dut =>
      tieOffAXISlave(dut)
      dut.io.src_addr.poke(0x1000.U)
      dut.io.dst_addr.poke(0.U)
      dut.io.length.poke(2.U)
      dut.io.start.poke(true.B)
      dut.clock.step(1)
      dut.io.start.poke(false.B)

      fork {
        serveAXIRead(dut, Seq(0xAAL, 0xBBL))
      }.join()

      waitForDone(dut)
      dut.io.done.expect(true.B)
    }
  }

  // ── Test 4 ────────────────────────────────────────────────────────────
  it should "write transferred data to scratchpad sequentially" in {
    test(new QuasarDMA(dmaP, spP, nastiP)) { dut =>
      // 1. Initial setup
      dut.io.axi.ar.ready.poke(false.B)
      dut.io.axi.r.valid.poke(false.B)

      dut.io.src_addr.poke(0x2000.U)
      dut.io.dst_addr.poke(5.U)
      dut.io.length.poke(3.U)
      dut.io.start.poke(true.B)
      dut.clock.step(1)
      dut.io.start.poke(false.B)

      //Wait for the DMA to issue the Address Read (AR)
      while(!dut.io.axi.ar.valid.peek().litToBoolean) {
        dut.clock.step(1)
      }
      dut.io.axi.ar.ready.poke(true.B)
      dut.clock.step(1)
      dut.io.axi.ar.ready.poke(false.B)

      //Drive Data beats and sample Scratchpad port in the SAME thread
      val dataToTransfer = Seq(0xAAAAL, 0xBBBBL, 0xCCCCL)
      val writtenAddrs = scala.collection.mutable.ListBuffer[Int]()
      val writtenData  = scala.collection.mutable.ListBuffer[Long]()

      for (i <- 0 until 3) {
        //Poke the AXI Data
        dut.io.axi.r.valid.poke(true.B)
        dut.io.axi.r.bits.data.poke(dataToTransfer(i).U)
        dut.io.axi.r.bits.last.poke((i == 2).B)

        //Since we are in the same thread, the simulator knows the
        //combinatorial path from r.valid to sp_port.en is stable.
        if (dut.io.sp_port.en.peek().litToBoolean) {
          writtenAddrs += dut.io.sp_port.addr.peek().litValue.toInt
          writtenData  += dut.io.sp_port.data.peek().litValue.toLong
        }

        dut.clock.step(1)
      }

      dut.io.axi.r.valid.poke(false.B)
      while(!dut.io.done.peek().litToBoolean) {
        dut.clock.step(1)
      }

      assert(writtenAddrs.toSeq == Seq(5, 6, 7), s"Got $writtenAddrs")
      assert(writtenData.toSeq == Seq(0xAAAAL, 0xBBBBL, 0xCCCCL), s"Got $writtenData")
    }
  }

  // ── Test 5 ────────────────────────────────────────────────────────────
  it should "return to idle and not busy after transfer" in {
    test(new QuasarDMA(dmaP, spP, nastiP)) { dut =>
      tieOffAXISlave(dut)
      dut.io.src_addr.poke(0x3000.U)
      dut.io.dst_addr.poke(0.U)
      dut.io.length.poke(2.U)
      dut.io.start.poke(true.B)
      dut.clock.step(1)
      dut.io.start.poke(false.B)

      fork {
        serveAXIRead(dut, Seq(0x1L, 0x2L))
      }.join()

      waitForDone(dut)
      dut.clock.step(1)  // one cycle after done pulse
      dut.io.busy.expect(false.B)
      dut.io.done.expect(false.B)  // done is a pulse, not a level
    }
  }

  // ── Test 6 ────────────────────────────────────────────────────────────
  it should "handle a single-beat transfer" in {
    test(new QuasarDMA(dmaP, spP, nastiP)) { dut =>
      tieOffAXISlave(dut)
      dut.io.src_addr.poke(0x4000.U)
      dut.io.dst_addr.poke(10.U)
      dut.io.length.poke(1.U)
      dut.io.start.poke(true.B)
      dut.clock.step(1)
      dut.io.start.poke(false.B)

      fork {
        serveAXIRead(dut, Seq(0xDEADL))
      }.join()

      waitForDone(dut)
      dut.io.done.expect(true.B)
    }
  }

  // ── Test 7 ────────────────────────────────────────────────────────────
  it should "not start without a start pulse" in {
    test(new QuasarDMA(dmaP, spP, nastiP)) { dut =>
      tieOffAXISlave(dut)
      dut.io.src_addr.poke(0x5000.U)
      dut.io.dst_addr.poke(0.U)
      dut.io.length.poke(4.U)
      dut.io.start.poke(false.B)
      dut.clock.step(5)
      // Should still be idle — no start was given
      dut.io.busy.expect(false.B)
      dut.io.axi.ar.valid.expect(false.B)
    }
  }
}