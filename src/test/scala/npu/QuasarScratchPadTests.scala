package npu

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec

class QuasarScratchpadTests extends AnyFlatSpec with ChiselScalatestTester {

  val p = QuasarScratchpadParams(depth = 64, width = 32)

  behavior.of("QuasarScratchpad")

  it should "reset to zero — read valid is low before any access" in {
    test(new QuasarScratchpad(p)) { dut =>
      dut.io.read.en.poke(false.B)
      dut.io.read.addr.poke(0.U)
      dut.io.write.en.poke(false.B)
      dut.io.write.addr.poke(0.U)
      dut.io.write.data.poke(0.U)
      dut.clock.step(1)
      dut.io.read.valid.expect(false.B)
    }
  }

  it should "write and read back a value" in {
    test(new QuasarScratchpad(p)) { dut =>
      // Write phase
      dut.io.write.en.poke(true.B)
      dut.io.write.addr.poke(5.U)
      dut.io.write.data.poke(0xdeadbeefL.U)
      dut.io.read.en.poke(false.B)
      dut.io.read.addr.poke(0.U)
      dut.clock.step(1)

      // Read phase — deassert write, assert read
      dut.io.write.en.poke(false.B)
      dut.io.read.en.poke(true.B)
      dut.io.read.addr.poke(5.U)
      dut.clock.step(1)

      // Data valid one cycle after read request
      dut.io.read.valid.expect(true.B)
      dut.io.read.data.expect(0xdeadbeefL.U)
    }
  }

  it should "write multiple locations and read them back" in {
    test(new QuasarScratchpad(p)) { dut =>
      val testData = Seq(0x11111111L, 0x22222222L,
        0x33333333L, 0x44444444L)

      // Write all locations
      for ((data, addr) <- testData.zipWithIndex) {
        dut.io.write.en.poke(true.B)
        dut.io.write.addr.poke(addr.U)
        dut.io.write.data.poke(data.U)
        dut.io.read.en.poke(false.B)
        dut.io.read.addr.poke(0.U)
        dut.clock.step(1)
      }

      // Read all locations back
      dut.io.write.en.poke(false.B)
      for ((data, addr) <- testData.zipWithIndex) {
        dut.io.read.en.poke(true.B)
        dut.io.read.addr.poke(addr.U)
        dut.clock.step(1)
        dut.io.read.valid.expect(true.B)
        dut.io.read.data.expect(data.U)
      }
    }
  }

  it should "support simultaneous read and write to different addresses" in {
    test(new QuasarScratchpad(p)) { dut =>
      // Pre-load address 3 with known value
      dut.io.write.en.poke(true.B)
      dut.io.write.addr.poke(3.U)
      dut.io.write.data.poke(0xABCDL.U)
      dut.io.read.en.poke(false.B)
      dut.io.read.addr.poke(0.U)
      dut.clock.step(1)

      // Simultaneously write addr 7 and read addr 3
      dut.io.write.en.poke(true.B)
      dut.io.write.addr.poke(7.U)
      dut.io.write.data.poke(0x1234L.U)
      dut.io.read.en.poke(true.B)
      dut.io.read.addr.poke(3.U)
      dut.clock.step(1)

      // Read result for addr 3 is valid
      dut.io.read.valid.expect(true.B)
      dut.io.read.data.expect(0xABCDL.U)
    }
  }

  it should "not produce valid read when read.en is low" in {
    test(new QuasarScratchpad(p)) { dut =>
      dut.io.read.en.poke(false.B)
      dut.io.read.addr.poke(0.U)
      dut.io.write.en.poke(false.B)
      dut.io.write.addr.poke(0.U)
      dut.io.write.data.poke(0.U)
      dut.clock.step(1)
      dut.io.read.valid.expect(false.B)
    }
  }

  behavior.of("QuasarDoubleBuf")

  it should "write to bank1 and read from bank0 when bank_sel=0" in {
    test(new QuasarDoubleBuf(p)) { dut =>
      // bank_sel=0: array reads bank0, DMA writes bank1
      // First write something to bank0 using bank_sel=1
      // (bank_sel=1 means DMA writes bank0)
      dut.io.bank_sel.poke(true.B)
      dut.io.dma_port.en.poke(true.B)
      dut.io.dma_port.addr.poke(10.U)
      dut.io.dma_port.data.poke(0xCAFEL.U)
      dut.io.array_port.en.poke(false.B)
      dut.io.array_port.addr.poke(0.U)
      dut.clock.step(1)

      // Switch to bank_sel=0: array now reads bank0
      dut.io.bank_sel.poke(false.B)
      dut.io.dma_port.en.poke(false.B)
      dut.io.array_port.en.poke(true.B)
      dut.io.array_port.addr.poke(10.U)
      dut.clock.step(1)

      dut.io.array_port.valid.expect(true.B)
      dut.io.array_port.data.expect(0xCAFEL.U)
    }
  }

  it should "write to bank0 and read from bank1 when bank_sel=1" in {
    test(new QuasarDoubleBuf(p)) { dut =>
      // bank_sel=0: DMA writes bank1
      dut.io.bank_sel.poke(false.B)
      dut.io.dma_port.en.poke(true.B)
      dut.io.dma_port.addr.poke(7.U)
      dut.io.dma_port.data.poke(0xBEEFL.U)
      dut.io.array_port.en.poke(false.B)
      dut.io.array_port.addr.poke(0.U)
      dut.clock.step(1)

      // Switch to bank_sel=1: array now reads bank1
      dut.io.bank_sel.poke(true.B)
      dut.io.dma_port.en.poke(false.B)
      dut.io.array_port.en.poke(true.B)
      dut.io.array_port.addr.poke(7.U)
      dut.clock.step(1)

      dut.io.array_port.valid.expect(true.B)
      dut.io.array_port.data.expect(0xBEEFL.U)
    }
  }

  it should "allow simultaneous DMA write and array read on different banks" in {
    test(new QuasarDoubleBuf(p)) { dut =>
      // Pre-load bank0 addr 5 (need bank_sel=1 for DMA to write bank0)
      dut.io.bank_sel.poke(true.B)
      dut.io.dma_port.en.poke(true.B)
      dut.io.dma_port.addr.poke(5.U)
      dut.io.dma_port.data.poke(0x1111L.U)
      dut.io.array_port.en.poke(false.B)
      dut.io.array_port.addr.poke(0.U)
      dut.clock.step(1)

      // Now bank_sel=0: array reads bank0 addr 5
      //                 DMA writes bank1 addr 9 simultaneously
      dut.io.bank_sel.poke(false.B)
      dut.io.dma_port.en.poke(true.B)
      dut.io.dma_port.addr.poke(9.U)
      dut.io.dma_port.data.poke(0x9999L.U)
      dut.io.array_port.en.poke(true.B)
      dut.io.array_port.addr.poke(5.U)
      dut.clock.step(1)

      // Array read from bank0 addr 5 is valid
      dut.io.array_port.valid.expect(true.B)
      dut.io.array_port.data.expect(0x1111L.U)
    }
  }
}