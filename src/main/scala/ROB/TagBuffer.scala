package ROB

import chisel3._
import chisel3.util._

// A free list of physical register tags. Rename requests tags for destination
// registers and returns them when instructions commit or are squashed.
//
// Implemented as a circular FIFO of RFTag values. At reset all physical
// registers are in the free list. Tags are allocated at dispatch and freed
// at commit (when the previous mapping for a register is replaced).
//
// On misprediction the buffer is restored to its committed state by replaying
// the commit sequence — freed tags re-enter the list as the ROB replays.
// Parameters:
//   NUM_ISSUE   — tags allocated per cycle (DEC_WIDTH)
//   NUM_COMMIT  — tags freed per cycle (DEC_WIDTH)
//   TAG_SIZE    — physical tag width (8 for 128 physical registers)
//   NUM_PHYS    — number of physical registers (128)

class TagBuffer(
  NUM_ISSUE  : Int = 4,
  NUM_COMMIT : Int = 4,
  TAG_SIZE   : Int = 8,
  NUM_PHYS   : Int = 128
) extends Module {

  val RF_TAG_SIZE = TAG_SIZE - 1  // 7 bits — strips the MSB special flag

  val io = IO(new Bundle {
    // Misprediction control
    val IN_mispr        = Input(Bool())
    val IN_mispredFlush = Input(Bool())

    // Issue — allocate tags for new destination registers
    val IN_issueValid       = Input(Vec(NUM_ISSUE, Bool()))
    val OUT_issueTags       = Output(Vec(NUM_ISSUE, UInt(RF_TAG_SIZE.W)))
    val OUT_issueTagsValid  = Output(Vec(NUM_ISSUE, Bool()))

    // Commit — free old physical registers that are no longer needed
    // IN_commitNewest: true when this commit is the newest mapping for rd
    //   (prevents freeing a tag that was already overwritten by a later instr)
    // IN_RAT_commitPrevTags: the previous committed tag being replaced
    // IN_commitTagDst: the tag being committed (needed to restore on mispr)
    val IN_commitValid        = Input(Vec(NUM_COMMIT, Bool()))
    val IN_commitNewest       = Input(Vec(NUM_COMMIT, Bool()))
    val IN_RAT_commitPrevTags = Input(Vec(NUM_COMMIT, UInt(TAG_SIZE.W)))
    val IN_commitTagDst       = Input(Vec(NUM_COMMIT, UInt(TAG_SIZE.W)))
  })


  // Circular FIFO. At reset physical registers 0..NUM_PHYS-1 are all free.
  // Head points to the next tag to allocate. Tail points to where the next
  // freed tag will be written.
  //
  // Two separate pointer pairs for speculative and committed state:
  //   specHead/specTail — tracks speculative allocations
  //   comHead/comTail   — tracks only committed allocations
  // On misprediction restore spec pointers to committed pointers.

  val FIFO_DEPTH = NUM_PHYS
  val ADDR_BITS  = log2Ceil(FIFO_DEPTH)

  val mem = RegInit(VecInit((0 until FIFO_DEPTH).map(_.U(RF_TAG_SIZE.W))))

  // Speculative pointers — updated at every dispatch and commit
  val specHead = RegInit(0.U((ADDR_BITS + 1).W))
  val specTail = RegInit(0.U((ADDR_BITS + 1).W))

  // Committed pointers — updated only at commit, used to restore on mispredict
  val comHead  = RegInit(0.U((ADDR_BITS + 1).W))
  val comTail  = RegInit(NUM_PHYS.U((ADDR_BITS + 1).W))

  // Number of free tags available speculatively
  val freeCount = WireDefault(specTail - specHead)

  // ---- Output Allocation -----------------------------------------------
  //
  // For each issue slot, provide the next available tag if one exists.
  // Tags are handed out speculatively from specHead forward.

  for (i <- 0 until NUM_ISSUE) {
    val offset = i.U
    val available = freeCount > offset

    io.OUT_issueTagsValid(i) := available
    io.OUT_issueTags(i)      := Mux(available,
      mem((specHead + offset)(ADDR_BITS - 1, 0)),
      0.U
    )
  }


  // Count how many tags are actually being issued this cycle
  val issueCount = PopCount(io.IN_issueValid)

  // Count how many tags are being freed this cycle
  // A tag is freed when:
  //   - commit is valid
  //   - this is the newest commit for this rd (not overwritten by later instr)
  //   - the previous tag is a real physical register (not a special tag)
  //   - not during misprediction replay (IN_mispredFlush without IN_mispr)
  val freeValids = Wire(Vec(NUM_COMMIT, Bool()))
  for (i <- 0 until NUM_COMMIT) {
    freeValids(i) := io.IN_commitValid(i) &&
                     io.IN_commitNewest(i) &&
                     !io.IN_RAT_commitPrevTags(i)(TAG_SIZE - 1) &&  // not special
                     !io.IN_mispredFlush
  }
  val freeCount_new = PopCount(freeValids)

  // On misprediction — restore speculative pointers to committed state
  when(io.IN_mispr) {
    specHead := comHead
    specTail := comTail
  }.otherwise {
    // Normal operation — advance head on allocation, tail on free

    // Advance specHead by number of tags issued
    when(issueCount > 0.U) {
      specHead := specHead + issueCount
    }

    // Write freed tags into the FIFO at specTail and advance tail
    var tailOffset = 0.U(ADDR_BITS.W)
    for (i <- 0 until NUM_COMMIT) {
      when(freeValids(i)) {
        mem((specTail + tailOffset)(ADDR_BITS - 1, 0)) :=
          io.IN_RAT_commitPrevTags(i)(RF_TAG_SIZE - 1, 0)
        tailOffset = tailOffset + 1.U
      }
    }
    when(freeCount_new > 0.U) {
      specTail := specTail + freeCount_new
    }
  }

  // Update committed pointers at commit — mirrors spec update but only
  // advances when instructions permanently retire (not during replay)
  when(!io.IN_mispredFlush) {
    // Committed head advances when tags are permanently allocated
    // This happens when the corresponding instruction commits
    // Track how many committed instructions consumed tags
    val comIssueCount = Wire(UInt((log2Ceil(NUM_COMMIT) + 1).W))
    comIssueCount := PopCount(
      VecInit((0 until NUM_COMMIT).map(i =>
        io.IN_commitValid(i) &&
        !io.IN_commitTagDst(i)(TAG_SIZE - 1)  // real physical tag was allocated
      ))
    )
    when(comIssueCount > 0.U) {
      comHead := comHead + comIssueCount
    }

    // Committed tail advances when old tags are permanently freed
    when(freeCount_new > 0.U) {
      comTail := comTail + freeCount_new
    }
  }
}
