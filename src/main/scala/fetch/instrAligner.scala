package fetch

import chisel3._
import chisel3.util._

//  IF_Packet — the fetch packet produced by IFetchPipeline
//  and consumed by InstrAligner.
//  Contains raw 16-bit half-words from the ICache line,
//  plus the metadata needed to reconstruct PD_Instr outputs.

class IF_Packet(NUM_HALFWORDS: Int) extends Bundle {
  val valid      = Bool()
  val pc         = UInt(32.W)
  val fetchID    = new FetchID
  val firstValid = new FetchOff      // index of first live half-word
  val lastValid  = new FetchOff      // index of last live half-word (inclusive)
  val fault      = IFetchFault()
  val halfWords  = Vec(NUM_HALFWORDS, UInt(16.W))
}


//  InstrAligner -- claude code
//
//  Converts a fetch packet of raw RISC-V half-words into up to
//  DEC_WIDTH decoded PD_Instr entries per cycle.
//
//  Handles:
//    - 16-bit compressed instructions  (bits[1:0] != 2'b11)
//    - 32-bit standard instructions    (bits[1:0] == 2'b11)
//    - firstValid / lastValid windowing
//    - Fault propagation (whole packet gets fault on IF_PAGE_FAULT etc.)
//    - Cross-packet 32-bit instruction spanning (carry register)
//
//  Parameters
//    NUM_PACKETS  : number of half-word slots in one fetch packet
//                  (= 1 << (FSIZE_E - 1), typically 8 for 16-byte lines)
//    DEC_WIDTH    : number of PD_Instr outputs per cycle (typically 4)

class InstrAligner(
  NUM_PACKETS : Int = 8,
  DEC_WIDTH   : Int = 4
) extends Module {

  val io = IO(new Bundle {
    val clear   = Input(Bool())
    val accept  = Input(Bool())           // downstream is ready to consume
    val ready   = Output(Bool())          // aligner is ready for a new packet

    val inPkt   = Input(new IF_Packet(NUM_PACKETS))

    val outReady = Input(Bool())          // decode stage ready
    val outInstrs = Output(Vec(DEC_WIDTH, new PD_Instr))
  })

  // ---- Carry register ----------------------------------------
  // When a 32-bit instruction straddles two fetch packets, the
  // lower 16 bits arrive in packet N and the upper 16 bits
  // arrive in packet N+1.  We save the lower half here.
  val carryValid = RegInit(false.B)
  val carryBits  = RegInit(0.U(16.W))
  val carryPC    = RegInit(0.U(32.W))
  val carryID    = RegInit(0.U.asTypeOf(new FetchID))

  // ---- Internal packet buffer --------------------------------
  // We accept one IF_Packet per cycle (when ready) and hold it
  // while draining DEC_WIDTH instructions per cycle.
  val bufValid     = RegInit(false.B)
  val buf          = RegInit(0.U.asTypeOf(new IF_Packet(NUM_PACKETS)))
  val drainPtr     = RegInit(0.U(log2Ceil(NUM_PACKETS + 1).W))

  io.ready := !bufValid || (drainPtr >= buf.lastValid.value && io.outReady)

  // ---- Accept new packet into buffer -------------------------
  when(io.clear) {
    bufValid    := false.B
    carryValid  := false.B
    drainPtr    := 0.U
  }.elsewhen(io.inPkt.valid && io.ready && io.accept) {
    bufValid := true.B
    buf      := io.inPkt
    drainPtr := io.inPkt.firstValid.value
  }.elsewhen(io.outReady && drainPtr > buf.lastValid.value) {
    bufValid := false.B
  }

  // ---- Combinational decode of up to DEC_WIDTH instrs --------
  // We walk the half-word array starting at drainPtr and emit
  // up to DEC_WIDTH instructions into io.outInstrs.

  val outWires = Wire(Vec(DEC_WIDTH, new PD_Instr))
  for (i <- 0 until DEC_WIDTH) {
    outWires(i) := 0.U.asTypeOf(new PD_Instr)
  }

  // Slot pointer: tracks which half-word we are examining for
  // each output slot.  We use a carry-chain approach in Chisel
  // by computing slot pointers as a running sum.
  //
  // slotPtr(i) = half-word index for output slot i
  // slotAdv(i) = 1 if slot i is a 16-bit instr, 2 if 32-bit
  val slotPtr = Wire(Vec(DEC_WIDTH + 1, UInt(log2Ceil(NUM_PACKETS + 2).W)))
  val slotAdv = Wire(Vec(DEC_WIDTH, UInt(2.W)))

  slotPtr(0) := drainPtr

  for (i <- 0 until DEC_WIDTH) {
    val ptr    = slotPtr(i)
    val inBuf  = bufValid && ptr <= buf.lastValid.value
    val fault  = buf.fault =/= IFetchFault.IF_FAULT_NONE

    // Half-words at ptr and ptr+1 (guard against out-of-range)
    val hw0 = Mux(ptr < NUM_PACKETS.U,
                buf.halfWords(ptr(log2Ceil(NUM_PACKETS) - 1, 0)), 0.U)
    val hw1 = Mux(ptr + 1.U < NUM_PACKETS.U,
                buf.halfWords((ptr + 1.U)(log2Ceil(NUM_PACKETS) - 1, 0)), 0.U)

    // Is this a compressed instruction?
    // RISC-V compressed: bits[1:0] != 2'b11
    val isCarry = carryValid && (i.U === 0.U)
    val rawBits = Mux(isCarry, Cat(hw0, carryBits), Cat(hw1, hw0))
    val isCompressed = Mux(isCarry,
      false.B,                          // carry is always upper half of 32-bit
      hw0(1, 0) =/= 3.U
    )

    slotAdv(i) := Mux(isCarry || !isCompressed, 2.U, 1.U)
    slotPtr(i + 1) := slotPtr(i) + slotAdv(i)

    // PC for this instruction
    // Each half-word slot is 2 bytes so pc = basePC + ptr*2
    val instrPC = Mux(isCarry,
      carryPC,
      buf.pc(31, log2Ceil(NUM_PACKETS) + 1) ##
        (ptr(log2Ceil(NUM_PACKETS) - 1, 0) ## 0.U(1.W))
    )

    outWires(i).valid      := inBuf || (isCarry && i.U === 0.U)
    outWires(i).bits       := Mux(fault, 0.U,
                                Mux(isCompressed,
                                  Cat(0.U(16.W), hw0),
                                  rawBits))
    outWires(i).pc         := instrPC
    outWires(i).fetchID    := Mux(isCarry, carryID, buf.fetchID)
    outWires(i).compressed := isCompressed
    outWires(i).fault      := Mux(inBuf || isCarry, buf.fault,
                                IFetchFault.IF_FAULT_NONE)
  }

  io.outInstrs := outWires

  // ---- Advance drain pointer ---------------------------------
  when(!io.clear && bufValid && io.outReady) {
    // Advance by the number of half-words consumed this cycle.
    // Count only the valid output slots.
    val consumed = slotAdv.zipWithIndex.map { case (adv, i) =>
      Mux(outWires(i).valid, adv, 0.U)
    }.reduce(_ +& _)

    drainPtr := drainPtr + consumed

    // Update carry: if the last valid slot ends at lastValid and
    // the last half-word of a 32-bit instruction is out of range,
    // save the lower half for the next packet.
    val lastSlotPtr = slotPtr(DEC_WIDTH)
    val lastHW      = lastSlotPtr - 1.U

    when(lastHW === buf.lastValid.value &&
         !outWires(DEC_WIDTH - 1).compressed &&
         buf.halfWords(buf.lastValid.value)(1, 0) === 3.U) {
      carryValid := true.B
      carryBits  := buf.halfWords(buf.lastValid.value)
      carryPC    := buf.pc(31, log2Ceil(NUM_PACKETS) + 1) ##
                    (buf.lastValid.value ## 0.U(1.W))
      carryID    := buf.fetchID
    }.otherwise {
      carryValid := false.B
    }
  }

  when(io.clear) {
    drainPtr   := 0.U
    carryValid := false.B
    bufValid   := false.B
  }
}
