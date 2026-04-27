package npu

import chisel3._
import chisel3.util._

/**
 * QuasarTileControllerIO: interface between the CSR file, DMA,
 * scratchpad double buffer, and systolic array.
 *
 * From CSR file:
 *   dispatch    — pulse: start a new NPU kernel
 *   npu_reset   — level: reset all state
 *   base_addr   — source address for DMA weight fetch
 *   out_addr    — scratchpad destination address for results
 *   shape_m     — M dimension of matrix (number of output rows)
 *   shape_n     — N dimension of matrix (number of output cols)
 *   cmd         — operation code (GEMM, RELU, etc.)
 *
 * To CSR file (status):
 *   npu_busy    — level: NPU is executing
 *   npu_done    — pulse: last dispatch completed
 *   npu_error   — level: error condition
 *
 * To DMA:
 *   dma_start   — pulse: begin DMA transfer
 *   dma_src     — source byte address
 *   dma_dst     — scratchpad word destination address
 *   dma_len     — number of words to transfer
 *
 * From DMA:
 *   dma_done    — pulse: DMA transfer complete
 *   dma_busy    — level: DMA is running
 *
 * To double buffer:
 *   bank_sel    — which bank the array reads from
 *
 * To systolic array:
 *   array_start — pulse: begin computation (loads weights, starts feed)
 *
 * From systolic array:
 *   array_valid — the bottom row's valid_out(0) — computation done
 *
 * 3D stacking / CXL note:
 *   In Phase 9 the DMA start path is replaced by a CXL.mem write
 *   notification — when the host writes tensor data directly via
 *   CXL.mem, the controller skips the DMA state and goes straight
 *   to sCompute. A cxl_write_valid input will be added here.
 */
class QuasarTileControllerIO(sp: QuasarScratchpadParams) extends Bundle {
  // from CSR file
  val dispatch  = Input(Bool())
  val npu_reset = Input(Bool())
  val base_addr = Input(UInt(32.W))
  val out_addr  = Input(UInt(sp.addrBits.W))
  val shape_m   = Input(UInt(16.W))
  val shape_n   = Input(UInt(16.W))
  val cmd       = Input(UInt(4.W))

  // to CSR file
  val npu_busy  = Output(Bool())
  val npu_done  = Output(Bool())
  val npu_error = Output(Bool())

  // to/from DMA
  val dma_start = Output(Bool())
  val dma_src   = Output(UInt(32.W))
  val dma_dst   = Output(UInt(sp.addrBits.W))
  val dma_len   = Output(UInt(16.W))
  val dma_done  = Input(Bool())
  val dma_busy  = Input(Bool())

  // to double buffer
  val bank_sel  = Output(Bool())

  // to/from systolic array
  val array_start = Output(Bool())
  val array_valid = Input(Bool())

  // Phase 9 CXL stub
  val cxl_write_valid = Input(Bool())  // tied to false.B until Phase 9
}

/**
 * QuasarTileController: four-state dispatch orchestrator.
 *
 * State machine:
 *
 *   sIdle ──dispatch──▶ sDMA ──dma_done──▶ sCompute ──array_valid──▶ sDone
 *     ▲                                                                  │
 *     └──────────────────────────────────────────────────────────────────┘
 *
 *   sIdle:
 *     npu_busy = false
 *     On dispatch pulse: latch base_addr, out_addr, shape, cmd
 *                        assert dma_start for one cycle
 *                        bank_sel := current_bank (DMA writes other bank)
 *                        transition to sDMA
 *
 *   sDMA:
 *     npu_busy = true
 *     dma_src  = base_addr_reg
 *     dma_dst  = 0 (write from start of inactive bank)
 *     dma_len  = shape_m * shape_n (total weights to fetch)
 *     Wait for dma_done pulse
 *     On dma_done: toggle bank_sel (array now reads freshly filled bank)
 *                  assert array_start for one cycle
 *                  transition to sCompute
 *
 *   sCompute:
 *     npu_busy = true
 *     Wait for array_valid pulse
 *     On array_valid: transition to sDone
 *
 *   sDone:
 *     npu_busy = false
 *     npu_done = true (one cycle pulse)
 *     transition to sIdle
 *
 *   npu_reset (level, any state):
 *     Return to sIdle immediately, clear all registers
 */
class QuasarTileController(val sp: QuasarScratchpadParams = QuasarScratchpadParams())
  extends Module {
  val io = IO(new QuasarTileControllerIO(sp))

  object State extends ChiselEnum {
    val sIdle, sDMA, sCompute, sDone = Value
  }

  import State._

  val state = RegInit(sIdle)

  // Latched dispatch parameters
  val base_addr_reg = RegInit(0.U(32.W))
  val out_addr_reg = RegInit(0.U(sp.addrBits.W))
  val shape_m_reg = RegInit(0.U(16.W))
  val shape_n_reg = RegInit(0.U(16.W))
  val cmd_reg = RegInit(0.U(4.W))
  val bank_reg = RegInit(false.B) // current active bank

  // Default output values
  io.npu_busy := state =/= sIdle && state =/= sDone
  io.npu_done := false.B
  io.npu_error := false.B
  io.dma_start := false.B
  io.dma_src := base_addr_reg
  io.dma_dst := 0.U
  io.dma_len := shape_m_reg * shape_n_reg
  io.bank_sel := bank_reg
  io.array_start := false.B

  switch(state) {
    is(sIdle) {
      when(io.dispatch) {
        base_addr_reg := io.base_addr
        out_addr_reg := io.out_addr
        shape_m_reg := io.shape_m
        shape_n_reg := io.shape_n
        cmd_reg := io.cmd

        io.dma_start := true.B
        state := sDMA
      }
    }

    is(sDMA) {
      when(io.dma_done) {
        io.array_start := true.B
        bank_reg := !bank_reg
        state := sCompute
      }
    }

    is(sCompute) {
      when(io.array_valid) {
        state := sDone
      }
    }

    is(sDone) {
      io.npu_done := true.B
      state := sIdle
    }
  }

  when(io.npu_reset) {
    state := sIdle
    bank_reg := false.B
  }
}