package npu

import chisel3._
import chisel3.util._

/**
 * MacPrecision: selects the arithmetic precision of the MAC.
 * INT8  — 8-bit signed inputs, 32-bit signed accumulator (Phase 1, fully implemented)
 * F16   — 16-bit float inputs (stub, Phase 3)
 * F32   — 32-bit float inputs (stub, Phase 3)
 *
 * For 3D stacking context: INT8 is the target precision for inference.
 * Weights stored in the TSV-connected HBM slice below each PE tile will
 * be INT8-quantized (matching Axiom's Q8_0 / Q4_K GGUF formats).
 * The 32-bit accumulator lives in the PE's local register file, never
 * crossing the TSV — only final outputs move between tiers.
 */
sealed trait MacPrecision
case object INT8 extends MacPrecision
case object F16  extends MacPrecision  // stub
case object F32  extends MacPrecision  // stub

/**
 * QuasarMACParams: compile-time configuration for one MAC unit.
 *
 * @param precision    arithmetic precision (see MacPrecision)
 * @param pipeDepth    pipeline depth in stages (fixed at 3 for now)
 * @param withSatGuard emit overflow detection logic if true
 */
case class QuasarMACParams(
                            precision:    MacPrecision = INT8,
                            pipeDepth:    Int          = 3,
                            withSatGuard: Boolean      = true
                          ) {
  require(pipeDepth == 3, "Phase 1 only supports pipeDepth=3")
}

/**
 * QuasarMACIO: IO bundle for a single MAC unit.
 *
 * Input data path:
 *   a_in, b_in   — operands for this cycle's multiply
 *   valid_in     — true when a_in/b_in carry real data
 *   clear        — reset accumulator at start of new dot product
 *
 * Output data path:
 *   result_out   — 32-bit accumulated result (valid when valid_out high)
 *   valid_out    — true when result_out is meaningful
 *   overflow     — sticky flag: accumulator exceeded INT32 safe range
 *
 * 3D stack stub port:
 *   mem_port     — placeholder for TSV-connected local SRAM interface
 *                  wired to DontCare in Phase 1, real in Phase 6
 */
class QuasarMACIO extends Bundle {
  // input
  val a_in      = Input(SInt(8.W))
  val b_in      = Input(SInt(8.W))
  val valid_in  = Input(Bool())
  val clear     = Input(Bool())

  // output
  val result_out = Output(SInt(32.W))
  val valid_out  = Output(Bool())
  val overflow   = Output(Bool())

  // 3D stack stub — TSV local memory port (Phase 6)
  val mem_port   = new QuasarTSVStubPort
}

/**
 * QuasarTSVStubPort: placeholder for the TSV-connected SRAM interface.
 *
 * In the final 3D-stacked architecture each PE tile has a dedicated
 * memory slice directly below it connected through Through-Silicon Vias.
 * The port here stubs that interface so the IO boundary is established
 * now and wiring doesn't change when we implement it for real in Phase 6.
 *
 * Fields:
 *   req_valid  — PE is requesting a memory access (driven by PE)
 *   req_addr   — 32-bit byte address into local SRAM slice
 *   req_data   — write data (ignored on reads)
 *   req_write  — true = write, false = read
 *   resp_valid — memory response is ready (driven by SRAM slice)
 *   resp_data  — read data returned from SRAM slice
 *
 * All outputs from the PE side are driven to DontCare in Phase 1.
 * All inputs to the PE side are tied to 0 in Phase 1.
 */
class QuasarTSVStubPort extends Bundle {
  // PE to SRAM (outputs from MAC's perspective)
  val req_valid = Output(Bool())
  val req_addr  = Output(UInt(32.W))
  val req_data  = Output(UInt(32.W))
  val req_write = Output(Bool())

  // SRAM → PE (inputs to MAC's perspective)
  val resp_valid = Input(Bool())
  val resp_data  = Input(UInt(32.W))
}

/**
 * QuasarMAC: 3-stage pipelined multiply-accumulate unit.
 *
 * Pipeline structure:
 *
 *   Stage 1 (Multiply):
 *     - Compute product = a_in * b_in (SInt(8) * SInt(8) = SInt(16))
 *     - Register product into prod_reg
 *     - Register valid_in into valid_s1
 *     - Register clear into clear_s1 (must travel with the data)
 *
 *   Stage 2 (Accumulate):
 *     - If clear_s1: acc_reg := sign-extended prod_reg
 *     - Else:        acc_reg := acc_reg + sign-extended prod_reg
 *     - Only update when valid_s1 is high
 *     - Register valid_s1 into valid_s2
 *     - Overflow detection: check if acc would exceed INT32 bounds
 *
 *   Stage 3 (Output register):
 *     - result_reg := acc_reg
 *     - valid_out  := valid_s2
 *     - overflow flag is sticky (once set, stays set until clear)
 *
 *   Stage 4 is essentially to counter the hardware clock delay given
 *   in the tests. When running the tests it ran through 4 clock cycles but
 *   theoretically we need only 3. So that is why the last clock cycle is to drive out
 *   the registers.
 *
 * Implementation hint for the accumulate stage:
 *   Use a 33-bit intermediate for overflow detection —
 *   extend both acc_reg and prod_reg to 33 bits, add them,
 *   then check if bit 32 differs from bit 31 of the result
 *   (sign overflow condition for two's complement).
 */
class QuasarMAC(val params: QuasarMACParams = QuasarMACParams()) extends Module {
  val io = IO(new QuasarMACIO)

  //Stage 1 registers
  val prod_reg = RegInit(0.S(16.W))
  val valid_s1 = RegInit(false.B)
  val clear_s1 = RegInit(false.B)

  //Stage 2 registers
  val acc_reg  = RegInit(0.S(32.W))
  val valid_s2 = RegInit(false.B)
  val clear_s2 = RegInit(false.B)

  // Stage 3 registers
  val result_reg = RegInit(0.S(32.W))
  val valid_s3   = RegInit(false.B)

  //Stage 4: output registers
  val result_out_reg = RegInit(0.S(32.W))
  val valid_out_reg  = RegInit(false.B)
  val overflow_reg   = RegInit(false.B)


  prod_reg := (io.a_in * io.b_in)(15, 0).asSInt
  valid_s1 := io.valid_in
  clear_s1 := io.clear

  val prod_ext = prod_reg.pad(32)
  valid_s2 := valid_s1
  clear_s2 := clear_s1

  when(valid_s1) {
    when(clear_s1) {
      acc_reg := prod_ext
    }.otherwise {
      acc_reg := acc_reg + prod_ext
    }
  }

  if (params.withSatGuard) {
    val acc_33   = acc_reg.pad(33)
    val prod_33  = prod_ext.pad(33)
    val sum_33   = (acc_33 + prod_33)(32, 0)
    val new_overflow = sum_33(32) =/= sum_33(31)
    when(valid_s1 && !clear_s1 && new_overflow) {
      overflow_reg := true.B
    }
    when(valid_s2 && clear_s2) {
      overflow_reg := false.B
    }
  }

  result_reg := acc_reg
  valid_s3   := valid_s2

  result_out_reg := result_reg
  valid_out_reg  := valid_s3

  io.result_out := result_out_reg
  io.valid_out  := valid_out_reg
  io.overflow   := overflow_reg

  io.mem_port.req_valid := DontCare
  io.mem_port.req_addr  := DontCare
  io.mem_port.req_data  := DontCare
  io.mem_port.req_write := DontCare
}