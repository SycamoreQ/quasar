package branch_pred

import chisel3._
import chisel3.util._
import fetch.{BPBackup, BPUpdate, BTUpdate, BranchTgtSpec, FetchBranchProv, FetchID, FetchLimit, FetchOff, PCFileEntry, PCFileReadReq, PredBranch, RecoveryInfo, RetAct, RetStackIdx, ReturnDecUpdate}

class BranchPredictor(NUM_IN: Int = 2 , params: branchParams) extends Module {

  val io = IO(new Bundle {
    val en          = Input(Bool())
    val stall        = Output(Bool())
    val mispr        = Input(new FetchBranchProv)
    val pcValid      = Input(Bool())
    val recovery     = Input(new RecoveryInfo)
    val fetchLimit   = Output(new FetchLimit)
    val fetchid      = Input(new FetchID)
    val comfetchid   = Input(new FetchID)
    val pc           = Output(UInt(31.W))
    val lastOffs     = Output(new FetchOff)
    val currRetAddr  = Output(UInt(31.W))
    val lateRetAddr  = Output(UInt(31.W))
    val rIdx         = Output(new RetStackIdx)
    val predBr       = Output(new PredBranch)
    val retDecUpdate = Output(new ReturnDecUpdate)
    val pcFileRead   = Output(new PCFileReadReq)
    val pcFileRData  = Input(new PCFileEntry)
    val BT_Updates   = Input(Vec(NUM_IN, new BTUpdate))
    val BP_Updates   = Input(new BPUpdate)
  })

  val pcReg      = RegInit((params.ENTRY_POINT >> 1).U(31.W))
  val pcRegNoInc = RegInit(0.U(31.W))
  val history    = RegInit(0.U(params.BHist_t_LEN.W))
  val ignorePred = RegInit(true.B)
  val recovery_r = Wire(new RecoveryInfo)

  //  BP Backup File
  //  Stores predictor state per fetchID so it can be restored
  //  on a misprediction.
  val fetchIDWidth = 8
  val bpFileDepth  = 1 << fetchIDWidth
  val bpFile       = SyncReadMem(bpFileDepth, new BPBackup)

  val BP_FileRAddr = WireInit(0.U.asTypeOf(new FetchID))
  val BP_FileRE    = WireInit(false.B)
  val BP_FileRData = bpFile.read(BP_FileRAddr.value, BP_FileRE)


  val btb = Module(new BranchTargetBuffer(
    btbEntries  = 1 << 10,
    btbTagSize  = 8
  ))
  btb.io.pcValid  := io.pcValid
  btb.io.pc       := pcReg
  val BTB_br      = btb.io.branch

  val tage = Module(new TagePredictor())
  tage.io.predValid   := io.pcValid
  tage.io.predAddr    := pcReg
  tage.io.predHistory := history

  val TAGE_taken   = tage.io.predTaken
  val TAGE_tageID  = tage.io.predTageID
  val TAGE_altPred = tage.io.altPred

  //  Return Stack
  val retStack = Module(new ReturnStack(
    SIZE   = 16,
    RQSIZE = 8
  ))
  retStack.io.valid        := io.pcValid
  retStack.io.fetchID      := io.fetchid
  retStack.io.comFetchID   := io.comfetchid
  retStack.io.lastPC       := pcRegNoInc
  retStack.io.branch       := io.predBr
  retStack.io.mispr        := io.mispr
  retStack.io.recoveryIdx  := recRIdx
  retStack.io.returnUpd    := io.retDecUpdate

  val RET_br    = retStack.io.predBr
  val RET_stall = retStack.io.stall
  val RET_idx   = retStack.io.curIdx

  io.stall       := RET_stall
  io.rIdx        := RET_idx
  io.currRetAddr := retStack.io.curRetAddr
  io.lateRetAddr := retStack.io.lateRetAddr

  //  BP Backup write — save predictor state for this fetchID
  val bp_backup = Wire(new BPBackup)
  bp_backup.history         := history
  bp_backup.rIdx            := RET_idx
  bp_backup.isRegularBranch := io.predBr.taken
  bp_backup.predTaken       := io.predBr.taken
  bp_backup.predOffs        := io.predBr.offs
  bp_backup.pred            := io.predBr.valid
  bp_backup.tageID          := TAGE_tageID
  bp_backup.altPred         := TAGE_altPred

  when(io.en1) {
    bpFile.write(io.fetchid.value, bp_backup)
  }

  //  BTB update — pick the first valid update from all ports
  val bt_update = WireInit(0.U.asTypeOf(new BTUpdate))
  for (i <- 0 until NUM_IN) {
    when(io.BT_Updates(i).valid) {
      bt_update := io.BT_Updates(i)
    }
  }
  btb.io.btUpdate := bt_update

  //  Recovery PC computation
  //  Reads the PC file to find the base PC of the mispredicted
  //  fetch block, then adjusts based on the recovery target spec.
  val recoveredPC = Wire(UInt(31.W))
  val pcFromFile  = io.pcFileRData.pc
  val fetchOffsBits = (new FetchOff).getWidth

  recoveredPC := Cat(
    pcFromFile(30, fetchOffsBits),
    io.recovery.fetchOffs.value
  )

  switch(io.recovery.tgtSpec) {
    is(BranchTgtSpec.BR_TGT_CUR32) {
      recoveredPC := recoveredPC - 1.U
    }
    is(BranchTgtSpec.BR_TGT_CUR16) {
      recoveredPC := recoveredPC
    }
    is(BranchTgtSpec.BR_TGT_NEXT) {
      recoveredPC := recoveredPC + 1.U
    }
    is(BranchTgtSpec.BR_TGT_MANUAL) {
      recoveredPC := recoveredPC
    }
  }

  //  Mispredict recovery history reconstruction
  val recHistory  = Wire(UInt(params.BHist_t_LEN.W))
  val tempHistory = Wire(UInt(params.BHist_t_LEN.W))
  tempHistory := BP_FileRData.history

  val recHistStep1 = Mux(
    BP_FileRData.pred && io.recovery.fetchOffs.value > BP_FileRData.predOffs.value,
    Cat(tempHistory(params.BHist_t_LEN - 2, 0), BP_FileRData.predTaken),
    tempHistory
  )

  val recHistStep2 = Mux(
    io.recovery.histAct === fetch.HistAct.HIST_PUSH ||
    io.recovery.histAct === fetch.HistAct.HIST_POP,
    Cat(recHistStep1(params.BHist_t_LEN - 2, 0), false.B),
    recHistStep1
  )

  recHistory := Mux(
    io.recovery.histAct === fetch.HistAct.HIST_APPEND_1,
    Cat(recHistStep2(params.BHist_t_LEN - 2, 0), true.B),
    recHistStep2
  )

  //  Recovery return stack index
  val recRIdx = Wire(new RetStackIdx)
  recRIdx := BP_FileRData.rIdx
  switch(io.recovery.retAct) {
    is(RetAct.RET_POP)  { recRIdx.value := BP_FileRData.rIdx.value - 1.U }
    is(RetAct.RET_PUSH) { recRIdx.value := BP_FileRData.rIdx.value + 1.U }
  }

  //  History for current lookup
  //  Updated speculatively on each predicted branch.
  val lookupHistory = Wire(UInt(params.BHist_t_LEN.W))
  lookupHistory := history

  when(io.recovery.valid) {
    lookupHistory := recHistory
  }.elsewhen(io.predBr.valid &&
             io.predBr.btype === BranchType.BT_BRANCH &&
             !io.predBr.dirOnly) {
    lookupHistory := Cat(history(params.BHist_t_LEN - 2, 0), io.predBr.taken)
  }

  //  History for update (commit-time)
  val updHistory = Wire(UInt(params.BHist_t_LEN.W))
  updHistory := BP_FileRData.history
  when(BP_FileRData.pred &&
       BP_FileRData.isRegularBranch &&
       bpUpdateActive.fetchoff.value > BP_FileRData.predOffs.value) {
    updHistory := Cat(BP_FileRData.history(params.BHist_t_LEN - 2, 0), BP_FileRData.predTaken)
  }

  //  TAGE write port (commit-time update)
  tage.io.writeValid   := bpUpdateActive.valid
  tage.io.writeAddr    := io.pcFileRData.pc
  tage.io.writeHistory := updHistory
  tage.io.writeTageID  := BP_FileRData.tageID
  tage.io.writeTaken   := bpUpdateActive.taken
  tage.io.writeAltPred := BP_FileRData.altPred
  tage.io.writePred    := BP_FileRData.predTaken

  //  Branch update FIFO
  //  Buffers incoming branch resolution results from the ROB.
  val updFIFO     = Module(new Queue(new BPUpdate, 4))
  updFIFO.io.enq.valid := io.BP_Updates.valid
  updFIFO.io.enq.bits  := io.BP_Updates

  val bpUpdate    = updFIFO.io.deq.bits
  val updFIFO_deq = WireInit(false.B)
  updFIFO.io.deq.ready := updFIFO_deq

  val bpUpdateActive = RegInit(0.U.asTypeOf(new BPUpdate))
  when(updFIFO_deq) {
    bpUpdateActive := bpUpdate
  }.otherwise {
    bpUpdateActive.valid := false.B
  }

  //  Fetch limit output
  //  Limits how far ahead the fetch engine can run relative to
  //  the last committed / pending branch update.
  io.fetchLimit       := 0.U.asTypeOf(new FetchLimit)
  io.fetchLimit.valid := false.B

  when(bpUpdate.valid) {
    io.fetchLimit.valid        := true.B
    io.fetchLimit.offs         := bpUpdate.fetchID.value.asTypeOf(new FetchOff)
  }.elsewhen(io.BP_Updates.valid) {
    io.fetchLimit.valid        := true.B
    io.fetchLimit.offs         := io.BP_Updates.fetchID.value.asTypeOf(new FetchOff)
  }

  //  PC file read control and update FIFO dequeue
  io.pcFileRead.addr  := DontCare
  io.pcFileRead.valid := false.B
  BP_FileRAddr        := 0.U.asTypeOf(new FetchID)
  BP_FileRE           := false.B
  updFIFO_deq         := false.B

  when(io.mispr.taken) {
    BP_FileRAddr := io.mispr.fetchid
    BP_FileRE    := true.B
    when(io.mispr.tgtspec =/= BranchTgtSpec.BR_TGT_MANUAL) {
      io.pcFileRead.addr  := io.mispr.fetchid
      io.pcFileRead.valid := true.B
    }
  }.elsewhen(bpUpdate.valid) {
    updFIFO_deq         := true.B
    BP_FileRAddr        := bpUpdate.fetchID
    BP_FileRE           := true.B
    io.pcFileRead.addr  := bpUpdate.fetchID
    io.pcFileRead.valid := true.B
  }

  //  PC output and prediction logic
  //  ignorePred suppresses predictions for one cycle after reset
  //  or after a misprediction while the pipeline is flushing.
  val pcOut      = WireInit(pcReg)
  val predBrOut  = WireInit(0.U.asTypeOf(new PredBranch))
  val lastOffsOut = WireInit(VecInit(Seq.fill((new FetchOff).getWidth)(true.B)).asUInt.asTypeOf(new FetchOff))

  when(!ignorePred) {
    when(recovery_r.valid && recovery_r.tgtSpec =/= BranchTgtSpec.BR_TGT_MANUAL) {
      pcOut := recoveredPC
    }.elsewhen(BTB_br.valid && BTB_br.btype =/= BranchType.BT_RETURN) {
      predBrOut         := BTB_br
      predBrOut.taken   := TAGE_taken
      predBrOut.multiple := !TAGE_taken && BTB_br.multiple

      when(TAGE_taken) {
        pcOut       := BTB_br.target
        lastOffsOut := BTB_br.offs
      }

      when(BTB_br.multiple && BTB_br.offs.value =/= lastOffsOut.value) {
        lastOffsOut.value := BTB_br.offs.value
        pcOut := Cat(
          pcRegNoInc(30, fetchOffsBits),
          Fill(fetchOffsBits, 1.U)
        )
      }
    }.elsewhen(BTB_br.valid &&
               BTB_br.btype === BranchType.BT_RETURN &&
               RET_br.valid) {
      predBrOut          := BTB_br
      predBrOut.taken    := true.B
      predBrOut.multiple := false.B
      predBrOut.target   := retStack.io.curRetAddr
      pcOut              := retStack.io.curRetAddr
      lastOffsOut        := BTB_br.offs
    }.otherwise {
      predBrOut.valid   := true.B
      predBrOut.btype   := BranchType.BT_BRANCH
      predBrOut.dirOnly := true.B
      predBrOut.taken   := TAGE_taken
    }
  }

  io.pc      := pcOut
  io.predBr  := predBrOut
  io.lastOffs := lastOffsOut

  //  Sequential updates
  recovery_r.valid := false.B

  when(io.pcValid) {
    pcReg      := Cat(pcOut(30, fetchOffsBits) + 1.U, 0.U(fetchOffsBits.W))
    pcRegNoInc := pcOut
    ignorePred := false.B
    history    := lookupHistory
  }.elsewhen(recovery_r.valid) {
    history := recHistory
    when(recovery_r.tgtSpec =/= BranchTgtSpec.BR_TGT_MANUAL) {
      pcReg := recoveredPC
    }
  }

  when(io.mispr.taken) {
    recovery_r.valid     := true.B
    recovery_r.tgtSpec   := io.mispr.tgtspec
    recovery_r.fetchid   := io.mispr.fetchid
    recovery_r.retAct    := io.mispr.retAct
    recovery_r.histAct   := io.mispr.histAct
    recovery_r.fetchOffs := io.mispr.fetchoffs
    pcReg      := Mux(
      io.mispr.tgtspec === BranchTgtSpec.BR_TGT_MANUAL,
      io.mispr.dst,
      pcReg
    )
    ignorePred := true.B
  }
}
