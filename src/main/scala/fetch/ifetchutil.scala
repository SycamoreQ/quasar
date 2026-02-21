package fetch

import chisel3._
import chisel3.util._
import common._
import _root_.circt.stage.ChiselStage

case class iFetchParams (
                          numUops: Int = 3,
                          numBlocks: Int = 8,
                          numBPUpd: Int = 2,
                          numBranchPorts: Int = 2,
                          decWidth: Int = 4,
                          resetDelay: Int = 16,
                          wfiDelay: Int = 4
                        )

class FetchID extends Bundle {
  val value = UInt(8.W);
}

class FetchOff extends Bundle{
  val value = UInt(4.W);
}

class RetStackIdx extends Bundle {
  val value = UInt(4.W);
}

object HistAct extends ChiselEnum{
  val HIST_NONE, HIST_PUSH, HIST_POP = Value;
}

object RetAct extends ChiselEnum {
  val RET_NONE, RET_PUSH, RET_POP = Value
}

object BranchTgtSpec extends ChiselEnum {
  val BR_TGT_NEXT, BR_TGT_COMPUTED = Value;
}

object IFetchFault extends ChiselEnum {
  val IF_FAULT_NONE, IF_INTERRUPT, IF_PAGE_FAULT, IF_ACCESS_FAULT = Value;
}

class BranchProv extends Bundle {
  val taken = Bool();
  val fetchid = new FetchID();
  val dst = Uint(8.W);
  val histAct = HistAct();
  val retAct =  RetAct();
  val fetchoffs = new FetchOff();
  val tgtspec = new BranchTgtSpec();
}

class FetchBranchProv extends Bundle {
  val taken = Bool();
  val fetchid = new FetchID();
  val histAct = HistAct();
  val retAct = RetAct();
  val fetchoffs = new FetchOff();
  val tgtspec = new BranchTgtSpec();
  val wfi = Bool();
}

class DecoderBranch extends Bundle {
  val taken = Bool();
  val fetchid = new FetchID();
  val fetchoff = new FetchOff;
  val wfi = Bool();
}

class BTUpdate extends Bundle {
  val valid = Bool()
  val pc = UInt(32.W)
  val target = UInt(32.W)
  val taken = Bool()
  val isBranch = Bool()
  val isCall = Bool()
  val isReturn = Bool()
}

class BPUpdate extends Bundle {
  val valid = Bool()
  val fetchID = new FetchID
  val taken = Bool()
  val branchMask = UInt(16.W)
}

class ReturnDecUpdate extends Bundle {
  val valid = Bool()
  val fetchID = new FetchID
  val offset = new FetchOff
  val retAddr = UInt(31.W)
}

class PredBranch extends Bundle {
  val taken = Bool()
  val target = UInt(31.W)
  val shouldFlush = Bool()
  val isBranch = Bool()
  val isCall = Bool()
  val isReturn = Bool()
}

class FetchLimit extends Bundle {
  val offs = new FetchOff
  val valid = Bool()
}

class PCFileReadReq extends Bundle {
  val valid = Bool()
  val addr = new FetchID
}

class PCFileReadReqTH extends Bundle {
  val valid = Bool()
  val prio = Bool()
  val addr = new FetchID
}

class PCFileEntry extends Bundle {
  val pc = UInt(32.W)
  val history = UInt(16.W)
  val retAddr = UInt(31.W)
}

class PD_Instr extends Bundle {
  val valid = Bool()
  val bits = UInt(32.W)
  val pc = UInt(32.W)
  val fetchID = new FetchID
  val compressed = Bool()
  val fault = IFetchFault()
}

class VirtMemState extends Bundle {
  val enabled = Bool()
  val mode = UInt(2.W)
  val asid = UInt(16.W)
  val ppn = UInt(22.W)
}

class PageWalk_Req extends Bundle {
  val valid = Bool()
  val addr = UInt(32.W)
  val write = Bool()
}

class PageWalk_Res extends Bundle {
  val valid = Bool()
  val pte = UInt(32.W)
  val fault = Bool()
}

class MemController_Req extends Bundle {
  val valid = Bool()
  val addr = UInt(32.W)
  val we = Bool()
  val wdata = UInt(32.W)
  val size = UInt(2.W)
}

class MemController_Res extends Bundle {
  val valid = Bool()
  val rdata = UInt(32.W)
  val fault = Bool()
}
