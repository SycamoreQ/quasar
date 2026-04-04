package LSU

import ROB.{ComLimit, R_UOp}
import branch_pred.{IS_UOp, SqN}
import chisel3._
import chisel3.util._
import decoder.{D_UOp, DecodeBranch, DecodeState, OpcodeConst}
import fetch.{BranchProv, IFetchFault, PD_Instr, iFetchParams}
import memory.MemController
import memory.AXIParams


class LoadBuffer(NUM_TFS: Int = 8 , NUM_TFS_IN: Int = 3 , WIDTH:4) (axi_params: AXIParams , fetch_params: iFetchParams) extends Module {
  val valid = Input(Bool())
  val in_memc = Input(new MemController(8 , 3 , axi_params , fetch_params))
  val in_commLoadSqN = Input(new SqN)
  val in_commStoreSqN = Input(new SqN)
  val in_stall = Input(Bool())
  val in_uop = Input(Vec(WIDTH, new R_UOp(4)))
  val in_branch = Input(new BranchProv)
  val in_sqDone = Input(Bool())


  val out_maxLoadSqN = Output(new SqN)
  val out_comLimit = Output(new ComLimit)
  val out_branch = Output(new BranchProv)


}