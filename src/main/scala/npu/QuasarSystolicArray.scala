package npu

import chisel3._
import chisel3.util._

/**
 * QuasarArrayParams: compile-time configuration for the systolic array.
 *
 * @param rows       number of PE rows (= number of output rows = M dimension)
 * @param cols       number of PE columns (= number of output cols = N dimension)
 * @param peParams   parameters forwarded to every PE in the array
 *
 * The array computes C[M x N] = A[M x K] * B[K x N] in a tiled fashion.
 * K (the inner dimension / dot product length) is not a compile-time
 * parameter — it is determined by how many valid activation cycles the
 * controller feeds into the array.
 *
 * 3D stacking note:
 *   In the target architecture, rows and cols determine the PE tile
 *   footprint on the compute die. Each PE tile sits above one SRAM
 *   slice in the memory die. The total on-chip weight capacity is:
 *     rows * cols * weightBits bits
 *   which is the amount of B-matrix data that fits in one tile pass.
 *   For a 4x4 array of INT8 PEs: 16 bytes per tile pass.
 *   For a 16x16 array: 256 bytes per tile pass.
 *   The tiling controller (Phase 4) loops over K to handle matrices
 *   larger than one tile.
 */
case class QuasarArrayParams(
                              rows:     Int           = 4,
                              cols:     Int           = 4,
                              peParams: QuasarPEParams = QuasarPEParams()
                            ) {
  require(rows > 0 && cols > 0, "Array dimensions must be positive")
  val actBits  = peParams.actBits
  val psumBits = peParams.psumBits
  val weightBits = peParams.weightBits
}

/**
 * WeightBundle: one weight value destined for a specific PE.
 * The weight load interface broadcasts a 2D grid of weights in one cycle.
 */
class WeightLoadIO(p: QuasarArrayParams) extends Bundle {
  // weight_data(i)(j) is loaded into PE[i][j] when weight_load is high
  val weight_load = Input(Bool())
  val weight_data = Input(Vec(p.rows, Vec(p.cols, SInt(p.weightBits.W))))
}

/**
 * QuasarSystolicArrayIO: top-level IO for the 4x4 (or NxM) array.
 *
 * Activation inputs (one per row, after external skewing):
 *   act_in(i)   — activation entering row i from the left boundary
 *                 The caller is responsible for skewing these inputs
 *                 by i cycles before presenting them here.
 *                 act_in(0) is presented immediately.
 *                 act_in(1) is delayed 1 cycle by the skew buffer.
 *                 act_in(2) is delayed 2 cycles. Etc.
 *
 * Control:
 *   valid_in    — all rows receive valid data this cycle
 *                 (single bit — all rows are fed synchronously after skew)
 *   clear       — start a new matrix tile (reset all PE accumulators)
 *   weights     — weight load interface (see WeightLoadIO)
 *
 * Outputs (one per column, from the bottom boundary):
 *   psum_out(j) — accumulated partial sum exiting column j downward
 *   valid_out   — high when psum_out values are meaningful
 *
 * Note on valid_out:
 *   valid_out goes high when the LAST row's valid signal has propagated
 *   through all PE stages. Since row (rows-1) is the deepest, valid_out
 *   is driven by the valid_out of PE[rows-1][0] (or any bottom PE,
 *   they are all synchronous).
 */
class QuasarSystolicArrayIO(p: QuasarArrayParams) extends Bundle {
  val act_in   = Input(Vec(p.rows, SInt(p.actBits.W)))
  val valid_in = Input(Bool())
  val clear    = Input(Bool())
  val weights  = new WeightLoadIO(p)
  val psum_out = Output(Vec(p.cols, SInt(p.psumBits.W)))
  val valid_out = Output(Vec(p.cols, Bool()))
}

/**
 * QuasarSystolicArray: NxM grid of weight-stationary PEs.
 *
 * Internal structure:
 *
 *   pes: Vec(rows, Vec(cols, QuasarPE))
 *     — a 2D array of PE modules
 *     — pes(i)(j) is the PE at row i, column j
 *
 * Wiring rules:
 *
 *   Horizontal (activation):
 *     pes(i)(0).io.act_in  ← act_in(i)   [left boundary]
 *     pes(i)(j).io.act_in  ← pes(i)(j-1).io.act_out  [interior]
 *
 *   Vertical (partial sum):
 *     pes(0)(j).io.psum_in ← 0.S   [top boundary: zero injection]
 *     pes(i)(j).io.psum_in ← pes(i-1)(j).io.psum_out  [interior]
 *
 *   Output:
 *     psum_out(j) ← pes(rows-1)(j).io.psum_out  [bottom boundary]
 *
 *   Valid and clear (broadcast to all PEs):
 *     Every PE receives the same valid_in and clear signals.
 *
 *   Weight loading (broadcast):
 *     pes(i)(j).io.weight_load ← weights.weight_load
 *     pes(i)(j).io.weight_data ← weights.weight_data(i)(j)
 *
 *   valid_out:
 *     Driven by pes(rows-1)(0).io.valid_out
 *     (bottom-left PE — all bottom PEs have same valid timing)
 *
 * TSV stub note:
 *   Each PE has a tsv_port. In Phase 6 these will be routed to a
 *   TSV interconnect fabric. For Phase 3, tie them all off with DontCare.
 */
class QuasarSystolicArray(val p: QuasarArrayParams = QuasarArrayParams()) extends Module {
  val io = IO(new QuasarSystolicArrayIO(p))

  // ── Instantiate PE grid ────────────────────────────────────────────────
  // Scala trick: use Seq.tabulate to create a 2D array of Modules
  // then convert to Vec for Chisel hardware indexing
  val pes = Seq.tabulate(p.rows, p.cols) { (i, j) =>
    Module(new QuasarPE(p.peParams))
  }

  val valid_skewed = Wire(Vec(p.cols, Bool()))
  val clear_skewed = Wire(Vec(p.cols, Bool()))

  // Column 0: no delay
  valid_skewed(0) := io.valid_in
  clear_skewed(0) := io.clear

  for (j <- 1 until p.cols) {
    val vReg = RegInit(false.B)
    val cReg = RegInit(false.B)
    vReg := valid_skewed(j-1)
    cReg := clear_skewed(j-1)
    valid_skewed(j) := vReg
    clear_skewed(j) := cReg
  }

  for (i <- 0 until p.rows) {
    for (j <- 0 until p.cols) {
      val pe = pes(i)(j)

      if (j == 0) {
        pe.io.act_in := io.act_in(i)
      } else {
        pe.io.act_in := pes(i)(j - 1).io.act_out
      }
      if (i == 0) {
        pe.io.psum_in := 0.S
      } else {
        pe.io.psum_in := pes(i - 1)(j).io.psum_out
      }

      pe.io.valid_in    := io.valid_in
      pe.io.clear       := io.clear
      pe.io.weight_load := io.weights.weight_load
      pe.io.weight_data := io.weights.weight_data(i)(j)
      pe.io.valid_in := valid_skewed(j)
      pe.io.clear    := clear_skewed(j)

      pe.io.tsv_port.resp_valid := false.B
      pe.io.tsv_port.resp_data  := 0.U
    }
  }

  for (j <- 0 until p.cols) {
    io.psum_out(j) := pes(p.rows - 1)(j).io.psum_out
  }

  for (j <- 0 until p.cols) {
    io.valid_out(j) := pes(p.rows-1)(j).io.valid_out
  }
}