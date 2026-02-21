package fetch;

import chisel3._
import chisel3.util._
import common._
import _root_.circt.stage.ChiselStage
import iFetchParams._;

class iFetch(p: iFetchParams) extends Module {
  val io = IO( new Bundle {
    val en = Input(Bool())
    val interruptPending = Input(Bool())
    val memBusy = Input(Bool())
    val rob_currfetchid = Input(new FetchID)
    val branch = Input(new BranchProv)
    val decBranch = Input(new DecoderBranch) 
    val clearCache = Input(Bool())
    val clearTlb = Input(Bool())
    val btUpdates = Input(Vec(params.numBPUpd, new BTUpdate))
    val bpUpdate =Input(new BPUpdate) 
    val pcRead = Input(Vec(p.numBranchPorts , new PCFileReadReq))
    val pcReadData = Output(Vec(params.numBranchPorts, new PCFileEntry))
    val pcReadTH = Input(new PCFileReadReqTH)
    val pcReadDataTH = Output(new PCFileEntry)
    val ready = Input(Bool())
    val instrs = Output(Vec(params.decWidth, new PD_Instr))
    val vmem = Input(new VirtMemState)
    val pw = new PageWalk_Req
    val pwRes = Flipped(new PageWalk_Res)
    val memc = new MemController_Req
    val memcRes = Flipped(new MemController_Res)
  } )
  
  val pc = Wire(Uint(31.W))
  val pcFull = Cat(pc, 0.U(1.W))
  pcFull 
}