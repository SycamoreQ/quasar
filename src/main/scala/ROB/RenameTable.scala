package rob

import chisel3._
import chisel3.util._

// ---------------------------------------------------------------------------
// RenameTable
//
// Maintains two mappings from architectural registers (x0-x31) to physical
// tags:
//
//   comTag  — committed mapping, updated only at ROB commit
//   specTag — speculative mapping, updated at dispatch, reset on mispredict
//
// Also tracks tag availability — whether a physical register has a valid
// result ready for reading.
//
// Parameters:
//   NUM_LOOKUP  — number of register lookups per cycle (2 * DEC_WIDTH)
//   NUM_ISSUE   — number of instructions dispatched per cycle (DEC_WIDTH)
//   NUM_COMMIT  — number of instructions committed per cycle (DEC_WIDTH)
//   NUM_WB      — number of writeback ports (DEC_WIDTH)
//   NUM_REGS    — number of architectural registers (32 for RV32I)
//   TAG_SIZE    — physical tag width in bits (8 for 128 physical registers)
//                 MSB = 1 means special tag (x0 or immediate, always available)
// ---------------------------------------------------------------------------
class RenameTable(
  NUM_LOOKUP  : Int = 8,    // 2 * DEC_WIDTH
  NUM_ISSUE   : Int = 4,    // DEC_WIDTH
  NUM_COMMIT  : Int = 4,    // DEC_WIDTH
  NUM_WB      : Int = 4,    // DEC_WIDTH
  NUM_REGS    : Int = 32,   // architectural register count
  TAG_SIZE    : Int = 8     // 128 physical registers + MSB special flag
) extends Module {

  val NUM_TAGS = 1 << (TAG_SIZE - 1)  // 128 physical registers

  val io = IO(new Bundle {
    // Misprediction control signals
    val IN_mispred      = Input(Bool())   // mispredict detected — reset specTag
    val IN_mispredFlush = Input(Bool())   // replay in progress — update specTag from commit

    // Lookup — combinational, returns speculative tag and availability
    val IN_lookupIDs       = Input(Vec(NUM_LOOKUP, UInt(5.W)))
    val OUT_lookupAvail    = Output(Vec(NUM_LOOKUP, Bool()))
    val OUT_lookupSpecTag  = Output(Vec(NUM_LOOKUP, UInt(TAG_SIZE.W)))

    // Issue — new instructions dispatched from rename
    val IN_issueValid = Input(Vec(NUM_ISSUE, Bool()))
    val IN_issueIDs   = Input(Vec(NUM_ISSUE, UInt(5.W)))
    val IN_issueTags  = Input(Vec(NUM_ISSUE, UInt(TAG_SIZE.W)))
    val IN_issueAvail = Input(Vec(NUM_ISSUE, Bool()))

    // Commit — from ROB, updates committed map
    val IN_commitValid     = Input(Vec(NUM_COMMIT, Bool()))
    val IN_commitIDs       = Input(Vec(NUM_COMMIT, UInt(5.W)))
    val IN_commitTags      = Input(Vec(NUM_COMMIT, UInt(TAG_SIZE.W)))
    // Previous committed tag returned so TagBuffer can free old physical reg
    val OUT_commitPrevTags = Output(Vec(NUM_COMMIT, UInt(TAG_SIZE.W)))

    // Writeback — execution units signal result availability
    val IN_wbValid = Input(Vec(NUM_WB, Bool()))
    val IN_wbTag   = Input(Vec(NUM_WB, UInt(TAG_SIZE.W)))
  })

  // ---- Storage ----------------------------------------------------------

  // Committed map — updated only at ROB commit, ground truth
  val comTag  = RegInit(VecInit(Seq.fill(NUM_REGS)(TagConst.TAG_ZERO)))

  // Speculative map — updated at dispatch, reset to comTag on mispredict
  val specTag = RegInit(VecInit(Seq.fill(NUM_REGS)(TagConst.TAG_ZERO)))

  // Tag availability — bit i set when physical register i has a valid result
  // All tags start available at reset
  // MSB of tag is the special flag — special tags are always available
  val tagAvail = RegInit(~0.U(NUM_TAGS.W))

  // ---- Combinational Lookup --------------------------------------------
  //
  // For each lookup port:
  //   1. Return current specTag and check tagAvail
  //   2. Forward from same-cycle writebacks
  //   3. Forward from earlier slots in the same dispatch group
  //      (slot i sees the results of slots 0..i/2-1)

  for (i <- 0 until NUM_LOOKUP) {
    // Base lookup from speculative map
    val baseTag   = specTag(io.IN_lookupIDs(i))
    val isSpecial = baseTag(TAG_SIZE - 1)  // MSB set = special tag, always available

    // Available if special tag OR physical register has a result
    val baseAvail = isSpecial || tagAvail(baseTag(TAG_SIZE - 2, 0))

    // Start with base values — forwarding below may override
    io.OUT_lookupSpecTag(i) := baseTag
    io.OUT_lookupAvail(i)   := baseAvail

    // Forward availability from same-cycle writebacks
    // If any writeback this cycle matches the looked-up tag, mark available
    for (j <- 0 until NUM_WB) {
      when(io.IN_wbValid(j) && io.IN_wbTag(j) === baseTag) {
        io.OUT_lookupAvail(i) := true.B
      }
    }

    // Forward from earlier dispatch slots in the same group
    // Slot i can see results from slots 0 .. (i/2) - 1
    // This handles RAW hazards within the same decode group
    for (j <- 0 until (i / 2)) {
      when(io.IN_issueValid(j) &&
           io.IN_issueIDs(j) === io.IN_lookupIDs(i) &&
           io.IN_issueIDs(j) =/= 0.U) {
        io.OUT_lookupAvail(i)   := io.IN_issueAvail(j)
        io.OUT_lookupSpecTag(i) := io.IN_issueTags(j)
      }
    }
  }

  // ---- Combinational Commit Previous Tag Lookup -----------------------
  //
  // Return the current committed tag for each commit ID before updating.
  // The TagBuffer uses this to know which old physical register to free.

  for (i <- 0 until NUM_COMMIT) {
    io.OUT_commitPrevTags(i) := comTag(io.IN_commitIDs(i))
  }

  // ---- Sequential Updates ---------------------------------------------

  always_ff {
    // ---- Writeback — mark physical registers as having valid results ----
    // Special tags (MSB=1) are never in the physical register file
    for (i <- 0 until NUM_WB) {
      when(io.IN_wbValid(i) && !io.IN_wbTag(i)(TAG_SIZE - 1)) {
        tagAvail(io.IN_wbTag(i)(TAG_SIZE - 2, 0)) := true.B
      }
    }

    // ---- Misprediction — reset specTag to comTag ----------------------
    // When a branch misprediction is detected, reset all speculative
    // mappings back to the last committed state. The ROB replay will
    // then re-apply committed changes via the commit ports below.
    when(io.IN_mispred) {
      for (i <- 1 until NUM_REGS) {  // x0 always stays TAG_ZERO
        specTag(i) := comTag(i)
      }
    }.otherwise {
      // ---- Issue — update speculative map for dispatched instructions --
      // Clear tagAvail for the new physical register being written
      for (i <- 0 until NUM_ISSUE) {
        when(io.IN_issueValid(i) && io.IN_issueIDs(i) =/= 0.U) {
          specTag(io.IN_issueIDs(i)) := io.IN_issueTags(i)

          // If the new tag is a real physical register (not special),
          // mark it unavailable until the execution unit writes back
          when(!io.IN_issueTags(i)(TAG_SIZE - 1)) {
            tagAvail(io.IN_issueTags(i)(TAG_SIZE - 2, 0)) := false.B
          }
        }
      }
    }

    // ---- Commit — update committed map --------------------------------
    // Also update specTag when mispredFlush is active (ROB replay)
    for (i <- 0 until NUM_COMMIT) {
      when(io.IN_commitValid(i) && io.IN_commitIDs(i) =/= 0.U) {
        when(io.IN_mispredFlush) {
          // During replay — update specTag to match what ROB is replaying
          // but only if we are not simultaneously resetting (IN_mispred)
          when(!io.IN_mispred) {
            specTag(io.IN_commitIDs(i)) := io.IN_commitTags(i)
          }
        }.otherwise {
          // Normal commit — update committed map
          comTag(io.IN_commitIDs(i)) := io.IN_commitTags(i)
          // If also mispredicting, bring specTag in line immediately
          when(io.IN_mispred) {
            specTag(io.IN_commitIDs(i)) := io.IN_commitTags(i)
          }
        }
      }
    }
  }
}
