package memory

import chisel3._
import chisel3.util._

object ArbState extends ChiselEnum {
  val sNone, sICache, sPageWalker = Value
}

// ---------------------------------------------------------------------------
// MemArbiter
//
// Round-robin arbiter with no preemption.
// Two clients — ICache and PageWalker — compete for one MemController port.
// Once a client wins the bus it holds it until the MemController signals
// the transaction is complete via memc_res.done (here: memc_res.valid &&
// !memc_res.burst, i.e. the last word of a burst or a single-word response).
//
// Arbitration policy:
//   - If only one client is requesting → grant that client
//   - If both are requesting → grant whoever was NOT served last
//   - If neither is requesting → bus stays idle
//   - Once granted → hold until memc_res indicates done
// ---------------------------------------------------------------------------
class MemArbiter extends Module {

  val io = IO(new Bundle {
    // Requests from clients
    val icache_req = Input(new MemController_Req)
    val pw_req     = Input(new MemController_Req)

    // Responses back to clients
    val icache_res = Output(new MemController_Res)
    val pw_res     = Output(new MemController_Res)

    // Single port to/from MemController
    val memc_req   = Output(new MemController_Req)
    val memc_res   = Input(new MemController_Res)

    val clear      = Input(Bool())
  })


  // owner_r      : who currently holds the bus
  // lastServed_r : who was served most recently — used for round-robin
  //                initialised to sPageWalker so ICache wins the very
  //                first simultaneous arbitration (gives fetch priority
  //                on startup without making ICache permanently higher)
  val owner_r      = RegInit(ArbState.sNone)
  val lastServed_r = RegInit(ArbState.sPageWalker)


  // nextOwner is a combinational wire — the grant decision this cycle.
  // If the bus is already held we do not re-arbitrate.
  // If the bus is free we apply round-robin selection.
  val nextOwner = Wire(ArbState())

  when(owner_r =/= ArbState.sNone) {
    // Bus is busy — hold current owner, no re-arbitration
    nextOwner := owner_r

  }.otherwise {
    // Bus is free — arbitrate
    when(!io.icache_req.valid && !io.pw_req.valid) {
      // Neither client is requesting
      nextOwner := ArbState.sNone

    }.elsewhen(io.icache_req.valid && !io.pw_req.valid) {
      // Only ICache is requesting
      nextOwner := ArbState.sICache

    }.elsewhen(io.pw_req.valid && !io.icache_req.valid) {
      // Only PageWalker is requesting
      nextOwner := ArbState.sPageWalker

    }.otherwise {
      // Both are requesting — grant whoever was NOT served last
      nextOwner := Mux(
        lastServed_r === ArbState.sICache,
        ArbState.sPageWalker,
        ArbState.sICache
      )
    }
  }

  // Forward the winning client's request to memc_req based on nextOwner.
  // Using nextOwner (not owner_r) means the winning request reaches the
  // MemController in the same cycle the grant is made — no extra latency.
  when(nextOwner === ArbState.sICache) {
    io.memc_req := io.icache_req
  }.elsewhen(nextOwner === ArbState.sPageWalker) {
    io.memc_req := io.pw_req
  }.otherwise {
    // Bus idle — drive memc_req to zero
    io.memc_req := 0.U.asTypeOf(new MemController_Req)
  }

  // Route memc_res back to the correct client based on owner_r.
  // We use owner_r (not nextOwner) because responses belong to whoever
  // currently owns the bus — the registered value, not the combinational
  // grant decision.
  // The non-owning client receives a zeroed response with valid = false.
  when(owner_r === ArbState.sICache) {
    io.icache_res := io.memc_res
    io.pw_res     := 0.U.asTypeOf(new MemController_Res)
  }.elsewhen(owner_r === ArbState.sPageWalker) {
    io.pw_res     := io.memc_res
    io.icache_res := 0.U.asTypeOf(new MemController_Res)
  }.otherwise {
    io.icache_res := 0.U.asTypeOf(new MemController_Res)
    io.pw_res     := 0.U.asTypeOf(new MemController_Res)
  }

  // Two updates happen at the clock edge:
  //
  // owner_r update:
  //   - If bus was free ,register the new grant from nextOwner
  //   - If bus was held and transaction is done , release to sNone
  //   - If bus was held and transaction still in progress → hold
  //
  // lastServed_r update:
  //   - Only updates when a NEW grant is made (bus was free and a
  //     client is now winning). Records the new owner so the OTHER
  //     client gets priority on the next simultaneous request.
  //
  // Transaction-done condition: memc_res.valid asserted and burst bit
  // is false, meaning this is the last (or only) word of the response.
  val txnDone = io.memc_res.valid && !io.memc_res.done

  when(owner_r === ArbState.sNone) {
    // Bus was free — register the new grant
    owner_r := nextOwner
    // Update lastServed only when a real grant is being made
    when(nextOwner =/= ArbState.sNone) {
      lastServed_r := nextOwner
      lastServed_r := nextOwner
    }
  }.elsewhen(txnDone) {
    // Transaction complete — release the bus
    owner_r := ArbState.sNone
  }
  // Otherwise: hold owner_r, burst still in progress

  // When clear is asserted (e.g. pipeline flush), release the bus
  // immediately and reset round-robin state.
  // lastServed_r resets to sPageWalker so ICache wins the next
  // simultaneous arbitration after the flush.
  when(io.clear) {
    owner_r      := ArbState.sNone
    lastServed_r := ArbState.sPageWalker
    io.memc_req  := 0.U.asTypeOf(new MemController_Req)
  }
}
