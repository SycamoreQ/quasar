package npu

import chisel3._

/**
 * QuasarPEParams: compile-time configuration for one Processing Element.
 *
 * @param macParams     parameters forwarded to the internal QuasarMAC
 * @param weightBits    bit width of the stationary weight register (default 8, INT8)
 * @param actBits       bit width of the activation flowing through (default 8, INT8)
 * @param psumBits      bit width of the partial sum bus (default 32)
 *
 * 3D stacking note:
 *   weightBits determines the width of the TSV read that fetches the
 *   stationary weight from the memory die below. In Phase 6 this
 *   becomes a full SRAM read; for now it is a register load.
 */
case class QuasarPEParams(
                           macParams:   QuasarMACParams = QuasarMACParams(),
                           weightBits:  Int             = 8,
                           actBits:     Int             = 8,
                           psumBits:    Int             = 32
                         )

/**
 * QuasarPEIO: IO bundle for one PE in the systolic array.
 *
 * Systolic data ports (these form the mesh connections):
 *
 *   act_in   — activation arriving from the LEFT neighbor (or array boundary)
 *   act_out  — activation passed to the RIGHT neighbor (registered copy of act_in)
 *   psum_in  — partial sum arriving from ABOVE (or zero at top boundary)
 *   psum_out — partial sum passed DOWNWARD after MAC accumulation
 *
 * Control ports:
 *   weight_load  — when high, latch weight_data into the stationary weight register
 *   weight_data  — the weight value to load (from tiling controller / TSV stub)
 *   valid_in     — marks act_in and psum_in as carrying real data this cycle
 *   clear        — forwarded to MAC: start a new dot product
 *
 * Output ports:
 *   valid_out    — follows valid_in through the PE pipeline
 *   overflow     — forwarded from MAC overflow flag
 *
 * 3D stack stub port (inherited concept from QuasarMAC):
 *   tsv_port     — placeholder for TSV weight fetch interface (Phase 6)
 */
class QuasarPEIO(p: QuasarPEParams) extends Bundle {
  // systolic data mesh
  val act_in    = Input(SInt(p.actBits.W))
  val act_out   = Output(SInt(p.actBits.W))
  val psum_in   = Input(SInt(p.psumBits.W))
  val psum_out  = Output(SInt(p.psumBits.W))

  // control
  val weight_load = Input(Bool())
  val weight_data = Input(SInt(p.weightBits.W))
  val valid_in    = Input(Bool())
  val clear       = Input(Bool())

  // status
  val valid_out   = Output(Bool())
  val overflow    = Output(Bool())

  // 3D stack stub — TSV weight fetch port (Phase 6)
  val tsv_port    = new QuasarTSVStubPort
}

/**
 * QuasarPE: one processing element in the Quasar systolic array.
 *
 * Internal structure:
 *
 *   weight_reg  — stationary weight, loaded once per matrix tile
 *   mac         — QuasarMAC instance, does the actual arithmetic
 *
 * Data flow this cycle (combinational intent, registered at MAC boundary):
 *
 *   1. act_in arrives from the left
 *   2. MAC computes weight_reg * act_in, accumulates into psum
 *      (the MAC's b_in = weight_reg, a_in = act_in)
 *   3. psum_in is ADDED to the MAC result to form psum_out
 *      Think of psum_in as the initial value of the accumulator
 *      coming from the PE above — but since our MAC handles its own
 *      accumulator internally, we add psum_in to the MAC's result_out
 *      at the output stage
 *   4. act_out = registered act_in (one cycle delay, systolic pass-through)
 *   5. valid_out follows valid_in through the pipeline
 *
 * IMPORTANT implementation note on psum:
 *   The MAC accumulates internally across multiple cycles (dot product).
 *   psum_in represents the partial sum from the PE above in the SAME
 *   cycle — it is added to result_out combinatorially, not fed into
 *   the MAC accumulator. This means:
 *     psum_out = mac.result_out + psum_in  (when valid_out is high)
 *     psum_out = psum_in                   (when valid_out is low,
 *                                           pass through unchanged)
 *
 * Weight loading:
 *   When weight_load is high, weight_reg latches weight_data.
 *   This happens independently of the data pipeline — you can load
 *   a new weight while the previous computation is still draining.
 *   In Phase 6 this load will be triggered by a TSV read completion.
 *
 * Hint for wiring the MAC:
 *   mac.io.a_in    = act_in  (activation is operand A)
 *   mac.io.b_in    = weight_reg  (weight is operand B)
 *   mac.io.valid_in = valid_in
 *   mac.io.clear    = clear
 *   The MAC's mem_port TSV stub should be connected to tsv_port
 *   so the port is visible at the PE boundary for Phase 6 wiring.
 */
class QuasarPE(val p: QuasarPEParams = QuasarPEParams()) extends Module {
  val io = IO(new QuasarPEIO(p))

  val weight_reg = RegInit(0.S(p.weightBits.W))

  val mac = Module(new QuasarMAC(p.macParams))

  val act_reg = RegInit(0.S(p.actBits.W))

  when (io.weight_load) {
    weight_reg := io.weight_data
  }

  mac.io.a_in := io.act_in
  mac.io.b_in := weight_reg
  mac.io.valid_in := io.valid_in
  mac.io.clear := io.clear

  mac.io.mem_port := DontCare

  act_reg := io.act_in
  io.act_out := act_reg


  when(mac.io.valid_out) {
    io.psum_out := mac.io.result_out + io.psum_in
  }.otherwise{
    io.psum_out := io.psum_in
  }

  io.valid_out := mac.io.valid_out
  io.overflow := mac.io.overflow
  io.tsv_port.req_valid := mac.io.mem_port.req_valid
  io.tsv_port.req_addr := mac.io.mem_port.req_addr
  io.tsv_port.req_data := mac.io.mem_port.req_data
  io.tsv_port.req_write := mac.io.mem_port.req_write
  mac.io.mem_port.resp_valid := io.tsv_port.resp_valid
  mac.io.mem_port.resp_data := io.tsv_port.resp_data

}