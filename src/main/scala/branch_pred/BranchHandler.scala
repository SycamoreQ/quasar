package branch_pred

import chisel3._
import chisel3.util._
import fetch.{BTUpdate, Branch, BranchProv, BranchTgtSpec, FetchID, FetchOff, HistAct, IFetchOp, RetAct, RetStackIdx}


/*
The BranchHandler sits between fetch and decode stages. It:

Parses raw instruction bits to find branches
Validates branch predictions made by the BTB/TAGE
Corrects wrong predictions early (in decode, before execute)
Updates the BTB with newly discovered branches
 */

class BranchHandler(NUM_INST: Int = 8) extends Module {
  val io = IO(new Bundle {
    val clear = Input(Bool())
    val accept = Input(Bool())
    val op = Input(new IFetchOp)
    val instrs = Input(Vec(NUM_INST, UInt(16.W)))

    val decBranch = Output(new FetchBranchProv)
    val btUpdate = Output(new BTUpdate)
    val retUpdate = Output(new ReturnDecUpdate)
    val endOffsValid = Output(Bool())
    val endOffs = Output(new FetchOff)
    val newPredTaken = Output(Bool())
    val newPredPos = Output(new FetchOff)
  })


  val lastInstr = RegInit(0.U(16.W))
  val lastInstrPC = RegInit(0.U(32.W))
  val lastInstrValid = RegInit(false.B)

  val instrsView = VecInit(io.instrs :+ lastInstr)   /// instruction bundle appending the last instruction
  val is16bit = Wire(Vec(NUM_INST + 1, Bool()))
  val is32bit = Wire(Vec(NUM_INST + 1, Bool()))

  val offsetWidth = log2Ceil((new FetchOff).getWidth)
  val firstValid = Mux(lastInstrValid, 0.U, io.op.pc(offsetWidth, 1))

  var validInstrStart = WireDefault(true.B)

  for (i <- 0 until NUM_INST) {
    is16bit(i) := false.B
    is32bit(i) := false.B
  }

  when(lastInstrValid && io.op.pc(offsetWidth, 1) === 0.U) {
    is32bit(0) := true.B
    validInstrStart = false.B
  }

  for (i <- 0 until NUM_INST + 1 ) {
    when ((i-1).U < firstValid) {
      validInstrStart := false.B
    }

    when((i-1).U === firstValid && !lastInstrValid){
       validInstrStart := true.B
    }

    when((i-1).U > io.op.lastValid.value){
      validInstrStart := true.B
    }

    when(validInstrStart) {
      when(instrsView(i)(1, 0) === 3.U) {
        is32bit(i) := true.B
        validInstrStart = false.B
      }.otherwise {
        is16bit(i) := true.B
        validInstrStart = true.B
      }
    }.otherwise {
      validInstrStart = true.B
    }
  }

  val pc = Wire(Vec(NUM_INST+1 , UInt(32.W)))
  pc(0) := lastInstrPC
  for(i <-0 until NUM_INST ) {
    pc(i) := Cat(io.op.pc(31, offsetWidth + 1), 0.U(offsetWidth.W), 0.U(1.W)) + ((i - 1) * 2).U
  }

  val CJ_target = Wire(Vec(NUM_INST + 1, UInt(32.W)))
  val CB_target = Wire(Vec(NUM_INST + 1, UInt(32.W)))
  val J_target = Wire(Vec(NUM_INST, UInt(32.W)))
  val B_target = Wire(Vec(NUM_INST, UInt(32.W)))


  /// init of the compressed jump and the compressed branch opcodes. They are 16 bit instrs.
  /// They are similar to the jal , j , bnez , beqz type of RISC instr which are 32 bit.

  /// This step essentially is the "predecode" wherein the algorithm uses the instrview as a high end view of the
  /// variable length instruction and pre decodes the instruction to find its length before actually decoding the opcode.
  for (i <- 0 until NUM_INST + 1) {
    val i16 = instrsView(i)

    CJ_target(i) := pc(i) + Cat(
      Fill(20, i16(12)), i16(12), i16(8), i16(10, 9), i16(6),
      i16(7), i16(2), i16(11), i16(5, 3), 0.U(1.W)
    )

    CB_target(i) := pc(i) + Cat(
      Fill(23, i16(12)), i16(12), i16(6, 5), i16(2),
      i16(11, 10), i16(4, 3), 0.U(1.W)
    )
  }

  for (i <- 0 until NUM_INST + 1) {
    val i32 = Cat(instrsView(i + 1), instrsView(i))

    // 32-bit Jump Target (JAL)
    // Mapping: imm[20], imm[10:1], imm[11], imm[19:12]
    J_target(i) := pc(i) + Cat(
      Fill(12, i32(31)), i32(19, 12), i32(20), i32(30, 21), 0.U(1.W)
    )

    B_target(i) := pc(i) + Cat(
      Fill(20, i32(31)), i32(7), i32(30, 25), i32(11, 8), 0.U(1.W)
    )
  }

  val branch = Wire(Vec(NUM_INST, new Branch))

  for (i <- 0 until NUM_INST ) {
    branch(i) := 0.U.asTypeOf(new Branch)
  }


  /// predecode and identification. Takes the funct (funct3 and funct7) parts of the instruction and checks if
  /// they are c.jal or c.j types therefore issuing a jump.
  for (i <- 0 until NUM_INST) {
    when(is16bit(i + 1)){
      val i16 = instrsView(i + 1)

      when((i16 & "b1110000000000011".U) === "b0010000000000001".U ||
        (i16 & "b1110000000000011".U) === "b1010000000000001".U) {
        branch(i).valid := true.B
        branch(i).compr := true.B
        branch(i).btype := Mux(i16(15, 13) === 1.U, BH_BranchType.CALL, BH_BranchType.JUMP)
        branch(i).target := CJ_target(i + 1)
        branch(i).pc := pc(i + 1)
        branch(i).fhPC := pc(i + 1)
      }

      when((i16 & "b1111000001111111".U) === "b1000000000000010".U && i16(11, 7) =/= 0.U) {
        branch(i).valid := true.B
        branch(i).compr := true.B
        branch(i).btype := Mux(i16(11, 7) === 1.U, BH_BranchType.RETURN, BH_BranchType.IJUMP)
        branch(i).target := DontCare
        branch(i).pc := pc(i + 1)
        branch(i).fhPC := pc(i + 1)
      }

      when((i16 & "b1111000001111111".U) === "b1001000000000010".U && i16(11, 7) =/= 0.U) {
        branch(i).valid := true.B
        branch(i).compr := true.B
        branch(i).btype := BH_BranchType.ICALL
        branch(i).target := DontCare
        branch(i).pc := pc(i + 1)
        branch(i).fhPC := pc(i + 1)
      }

      when((i16 & "b1110000000000011".U) === "b1110000000000001".U ||
        (i16 & "b1110000000000011".U) === "b1100000000000001".U) {
        branch(i).valid := true.B
        branch(i).compr := true.B
        branch(i).btype := BH_BranchType.BRANCH
        branch(i).target := CB_target(i + 1)
        branch(i).pc := pc(i + 1)
        branch(i).fhPC := pc(i + 1)
      }
    }
  }

  /// same with branch types like c.beq , c.bnez
  for (i <- 0 until NUM_INST) {
    when(is32bit(i)) {
      val i32 = Cat(instrsView(i + 1), instrsView(i))

      when((i32 & "b00000000000000000000000001111111".U) === "b00000000000000000000000001101111".U) {
        branch(i).valid := true.B
        branch(i).compr := false.B
        branch(i).btype := Mux(i32(11, 7) === 1.U || i32(11, 7) === 5.U, BH_BranchType.CALL, BH_BranchType.JUMP)
        branch(i).target := J_target(i)
        branch(i).pc := pc(i)
        branch(i).fhPC := pc(i + 1)
      }

      when((i32 & "b00000000000000000111000001111111".U) === "b00000000000000000000000001100111".U) {
        branch(i).valid := true.B
        branch(i).compr := false.B
        val isRdLink = i32(11, 7) === 1.U || i32(11, 7) === 5.U
        val isRs1Link = i32(19, 15) === 1.U || i32(19, 15) === 5.U
        val sameBit = i32(17) === i32(9)

        when(!isRdLink && !isRs1Link) {
          branch(i).btype := BH_BranchType.IJUMP
        }.elsewhen(!isRdLink && isRs1Link) {
          branch(i).btype := BH_BranchType.RETURN
        }.elsewhen(isRdLink && !isRs1Link) {
          branch(i).btype := BH_BranchType.ICALL
        }.elsewhen(isRdLink && isRs1Link && !sameBit) {
          branch(i).btype := BH_BranchType.IJUMP
        }.otherwise {
          branch(i).btype := BH_BranchType.ICALL
        }

        branch(i).target := DontCare
        branch(i).pc := pc(i)
        branch(i).fhPC := pc(i + 1)
      }

      when((i32 & "b00000000000000000000000001111111".U) === "b00000000000000000000000001100011".U &&
        instrsView(i)(14, 12) =/= 2.U && instrsView(i)(14, 12) =/= 3.U) {
        branch(i).valid := true.B
        branch(i).compr := false.B
        branch(i).btype := BH_BranchType.BRANCH
        branch(i).target := B_target(i)
        branch(i).pc := pc(i)
        branch(i).fhPC := pc(i + 1)
      }
    }
  }

  val decBranch_c = Wire(new FetchBranchProv)
  val btUpdate_c = Wire(new BTUpdate)
  val retUpd_c = Wire(new ReturnDecUpdate)
  val endOffsValid = Wire(Bool())
  val endOffs = Wire(new FetchOff)
  val newPredPos_c = Wire(new FetchOff)
  val newPredTaken_c = Wire(Bool())

  decBranch_c := 0.U.asTypeOf(new FetchBranchProv)
  decBranch_c.taken := false.B
  decBranch_c.isFetchBranch := true.B

  btUpdate_c := 0.U.asTypeOf(new BTUpdate)
  btUpdate_c.valid := false.B

  retUpd_c := 0.U.asTypeOf(new ReturnDecUpdate)
  retUpd_c.valid := false.B

  endOffs := DontCare
  endOffsValid := false.B

  newPredTaken_c := io.op.predBr.valid && io.op.predBr.taken
  newPredPos_c := io.op.predBr.offs


  for (i <- 0  until NUM_INST) {
    val curBr = branch(i)
    val curBr_btypeSimple = curBr.btype.asUInt(2, 1).asTypeOf(BranchType())

    val indirBranch = curBr.valid && (
      curBr.btype === BH_BranchType.IJUMP || curBr.btype ===  BH_BranchType.ICALL || curBr.btype === BH_BranchType.RETURN
    )

    val curPC = Cat(io.op.pc(31, offsetWidth + 1), i.U(offsetWidth.W))
    val nextPC = curPC + 1.U

    val predicted = io.op.predBr.valid && io.op.predBr.offs.value === i.U && !io.op.predBr.dirOnly

    /// masking or retrieving the right window of instructions. A lower bound to not check anything lesser than the offset
    /// upper bound for not checking beyond what is being fetched , and to check if we only care about the branch taken and not
    /// anything else in the superscalar slots
    when (decBranch_c.taken &&
          i.U <= io.op.lastValid.value  &&
          i.U >= io.op.pc(offsetWidth , 1)){
      /// predictor guesses taken , but its actually a misprediction , or a jump type , or target addr s wrong
      when (predicted && io.op.predBr.taken) {
        when (!curBr.valid ||
          curBr_btypeSimple =/= io.op.predBr.btype ||
          (!indirBranch && curBr.target(1 , 31) =/= io.op.predBr.target)
        ) {
          val predIllegal = is32bit(io.op.predBr.offs.value + 1.U)

          /* "flushes" the bad prediction and informs the Branch Target Buffer (BTB) to update its records
              because it was hallucinating a branch where there wasn't a correct one.
          */
          btUpdate_c.valid := true.B
          btUpdate_c.clean := true.B
          btUpdate_c.multiple := false.B
          btUpdate_c.fetchStartOffs := io.op.pc(offsetWidth, 1).asTypeOf(new FetchOff)
          btUpdate_c.source := Cat(io.op.pc(31, offsetWidth), io.op.predBr.offs.value)

          decBranch_c.taken := true.B
          decBranch_c.fetchID := io.op.fetchid
          decBranch_c := false.B
          decBranch_c.fetchOffs.value := 1.U
          decBranch_c.retAct := RetAct.RET_NONE
          decBranch_c.histAct := Mux(curBr.valid && curBr.btype === BH_BranchType.BRANCH,
            HistAct.HIST_APPEND_1, HistAct.HIST_NONE)
          decBranch_c.tgtSpec := BranchTgtSpec.BR_TGT_MANUAL
          decBranch_c.wfi := false.B

          /// if valid prediction , updates the Return Address Stack (RAS) of the address and nextPC
          when(curBr.valid && (curBr.btype === BH_BranchType.CALL || curBr.btype === BH_BranchType.ICALL)) {
            retUpd_c.valid := true.B
            retUpd_c.idx := io.op.rIdx
            retUpd_c.addr := Cat(io.op.pc(31, offsetWidth + 1), curBr.fhPC(offsetWidth, 1))
          }

          when(curBr.valid) {
            when(curBr.btype === BH_BranchType.CALL || curBr.btype === BH_BranchType.ICALL) {
              decBranch_c.retAct := RetAct.RET_PUSH
            }.elsewhen(curBr.btype === BH_BranchType.RETURN) {
              decBranch_c.retAct := RetAct.RET_POP
            }
          }
          /// handling indirect branches like ICALL , IJUMP , RETURN
          when(!predIllegal && curBr.valid && (!indirBranch || curBr.btype === BH_BranchType.RETURN)) {
            btUpdate_c.clean := false.B
            btUpdate_c.btype := curBr_btypeSimple
            btUpdate_c.src := curBr.fhPC
            btUpdate_c.fetchStartOffs := io.op.pc(offsetWidth, 1).asTypeOf(new FetchOff)
            btUpdate_c.dst := curBr.target
            btUpdate_c.compressed := curBr.compr

            decBranch_c.dst := Mux(curBr.btype === BH_BranchType.RETURN,
              io.op.predRetAddr, curBr.target(31, 1))
          }.elsewhen(predIllegal) {
            decBranch_c.dst := curBr.pc
            newPredTaken_c := false.B
            newPredPos_c.value := Fill(offsetWidth, 1.U)
            endOffsValid := true.B
            endOffs := io.op.predBr.offs
          }.otherwise {
            decBranch_c.dst := nextPC
            newPredTaken_c := false.B
            newPredPos_c.value := Fill(offsetWidth, 1.U)

            when(io.op.predBr.offs.value =/= Fill(offsetWidth, 1.U)) {
              endOffsValid := true.B
              endOffs.value := io.op.predBr.offs.value + 1.U
            }
          }
        }.elsewhen(curBr.valid){ /// this part handles not taken branches and direction only predictions
          val actualOffs = curBr.fhPC(offsetWidth , 1).asTypeOf(new FetchOff)
          val dirOnly = curBr.btype === BH_BranchType.BRANCH &&
            io.op.predBr.valid && io.op.predBr.dirOnly
          val dirOnlyTaken = dirOnly && io.op.predBr.taken

          when (curBr.btype === BH_BranchType.JUMP ||
                curBr.btype === BH_BranchType.CALL ||
                curBr.btype === BH_BranchType.RETURN ||
                dirOnlyTaken) {
            decBranch_c.taken := true.B
            decBranch_c.fetchID := io.op.fetchid
            decBranch_c.fetchOffs.value := i.U
            decBranch_c.target := Mux(curBr.btype === BH_BranchType.RETURN,
              io.op.predRetAddr, curBr.target(31, 1))
            decBranch_c.histAct := Mux(dirOnlyTaken, HistAct.HIST_APPEND_1, HistAct.HIST_NONE)
            decBranch_c.tgtSpec := BranchTgtSpec.BR_TGT_MANUAL
            decBranch_c.wfi := false.B

            when (curBr.btype === BH_BranchType.CALL || curBr.btype === BH_BranchType.ICALL){
              retUpd_c.valid := true.B
              retUpd_c.idx := io.op.rIdx
              retUpd_c.addr := Cat(io.op.pc(31 , offsetWidth + 1) , actualOffs.value)
            }.elsewhen(predicted) {
              val predIllegal = is32bit(io.op.predBr.offs.value + 1.U)
              decBranch_c.taken := true.B
              decBranch_c.fetchID := io.op.fetchid
              decBranch_c.fetchOffs.value := i.U
              decBranch_c.dst := nextPC
              decBranch_c.retAct := RetAct.RET_NONE
              decBranch_c.histAct := HistAct.HIST_NONE
              decBranch_c.tgtSpec := BranchTgtSpec.BR_TGT_MANUAL
              decBranch_c.wfi := false.B

              /* predictor was tracking a branch but the instruction at that slot turned out to be "Illegal" or not a branch
                 tell the algorithm to follow the next pc and not the ghost prediction.
              */
              newPredTaken_c := false.B
              newPredPos_c.value := Fill(offsetWidth, 1.U)

              when(i.U =/= Fill(offsetWidth, 1.U)) {
                endOffsValid := true.B
                endOffs.value := i.U + 1.U
              }
              ///
              when(predIllegal) {
                endOffsValid := true.B
                endOffs.value := i.U
                decBranch_c.target := curPC
              }

              btUpdate_c.valid := true.B
              btUpdate_c.clean := true.B
              btUpdate_c.multiple := false.B
              btUpdate_c.fetchStartOffs := io.op.pc(offsetWidth, 1).asTypeOf(new FetchOff)
              btUpdate_c.source := Cat(io.op.pc(31, offsetWidth), io.op.predBr.offs.value)

            }
          }
        }
      }
    }
    io.decBranch := Mux(io.accept, decBranch_c, 0.U.asTypeOf(new FetchBranchProv))
    io.retUpdate := Mux(io.accept, retUpd_c, 0.U.asTypeOf(new ReturnDecUpdate))
    io.endOffsValid := Mux(io.accept, endOffsValid, false.B)
    io.endOffs := Mux(io.accept, endOffs, DontCare)
    io.newPredPos := Mux(io.accept, newPredPos_c, DontCare)
    io.newPredTaken := Mux(io.accept, newPredTaken_c, false.B)

    val btUpdate_r = RegInit(0.U.asTypeOf(new BTUpdate))
    btUpdate_r := 0.U.asTypeOf(new BTUpdate)
    when(io.accept && !io.clear) {
      btUpdate_r := btUpdate_c
    }
    io.btUpdate := btUpdate_r

    when(io.clear) {
      lastInstr := DontCare
      lastInstrValid := false.B
    }.elsewhen(io.accept) {
      val lastIdx = io.op.lastValid.value + 1.U

      when(is32bit(lastIdx) && !decBranch_c.taken) {
        lastInstrValid := true.B
        lastInstr := instrsView(NUM_INST)
        lastInstrPC := pc(NUM_INST)
      }.otherwise {
        lastInstrValid := false.B
        lastInstr := DontCare
      }
    }
  }
}

  





