package huancun.prefetch

import huancun.utils.SRAMTemplate
import huancun.utils.ReplaceableQueue
import huancun.utils.ReplaceableQueueV2
import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import huancun.HasHuanCunParameters
import utility.ChiselDB
import huancun.utils.XSPerfAccumulate

case class StreamerParameters(
  sTableEntries: Int = 32,
  dist_within_fb: Int = 8,
  dist_tw_home: Int = 3,
  dist_fb_home: Int = 5,
  dist_demand_l2home: Int = 1,
  degree: Int = 5
)
    extends PrefetchParameters {
  override val hasPrefetchBit:  Boolean = true
  override val inflightEntries: Int = 32
}

trait HasStreamerParams extends HasHuanCunParameters {
  val streamerParams = prefetchOpt.get.asInstanceOf[StreamerParameters]

  val sTableEntries = streamerParams.sTableEntries
  val fTableEntries = streamerParams.sTableEntries
  val dist_within_fb = streamerParams.dist_within_fb
  val dist_tw_home = streamerParams.dist_tw_home
  val dist_fb_home = streamerParams.dist_fb_home
  val dist_demand_l2home = streamerParams.dist_demand_l2home
  val degree = streamerParams.degree

  val inflightEntries = streamerParams.inflightEntries
  val pageAddrBits = fullAddressBits - pageOffsetBits
  val blkOffsetBits = pageOffsetBits - offsetBits

  val sTagBits = pageAddrBits - log2Up(sTableEntries)
}

abstract class StreamerBundle(implicit val p: Parameters) extends Bundle with HasStreamerParams
abstract class StreamerModule(implicit val p: Parameters) extends Module with HasStreamerParams

class StreamerTableUpdate(implicit p: Parameters) extends StreamerBundle {
  val pageAddr = UInt(pageAddrBits.W)
  val blkOffset = UInt(blkOffsetBits.W)
}

class sTableEntry(implicit p: Parameters) extends StreamerBundle {
  val valid = Bool()
  val tag = UInt(sTagBits.W)
  val l2_home = UInt(blkOffsetBits.W)
  val pendding = UInt(blkOffsetBits.W)
  val lastBlock = UInt(blkOffsetBits.W)
  val state = UInt(2.W)
  val farBack = Bool()
}

class idxEntry(implicit p: Parameters) extends StreamerBundle {
  val valid = Bool()
  val index = UInt(log2Up(sTableEntries).W)

  def apply(v: Bool, i: UInt) = {
    val entry = Wire(this)
    entry.valid := v
    entry.index := i
    entry
  }

  def apply() = {
    val entry = Wire(this)
    entry.valid := false.B
    entry.index := 0.U
    entry
  }
}

class StreamerTable(implicit p: Parameters) extends StreamerModule {
  val io = IO(new Bundle {
    val update = Flipped(DecoupledIO(new StreamerTableUpdate))
    val issue = DecoupledIO(UInt((pageAddrBits + blkOffsetBits).W))
  })

  val s_done :: s_searching :: s_forward :: s_backward :: Nil = Enum(4)

  def idx(addr:      UInt) = addr(log2Up(sTableEntries) - 1, 0)
  def tag(addr:      UInt) = addr(pageAddrBits - 1, log2Up(sTableEntries))
  
  val sTable = Module(
    new SRAMTemplate(new sTableEntry, set = sTableEntries, way = 1, bypassWrite = true, shouldReset = true)
  )

  val pageAddr = io.update.bits.pageAddr
  val pageAddr_l = RegNext(pageAddr)
  val block = io.update.bits.blkOffset
  val block_l = RegNext(block)
  // val countOn = !io.update.fire()
  // val counter = Counter(countOn, sTableEntries)
  val valids = RegInit(VecInit.tabulate(sTableEntries){x => false.B})
  val lastValidIndex = RegInit(0.U(log2Up(sTableEntries).W))
  val leftValidIndex = valids.zipWithIndex.map{ case(v, i) => (new idxEntry).apply(v, i.asUInt)}.reduce((a, b) => 
                        Mux(a.valid === true.B && a.index <= lastValidIndex, a, b)).index
  val rightValidIndex = valids.zipWithIndex.map{ case(v, i) => (new idxEntry).apply(v, i.asUInt)}.reduce((a, b) => 
                        Mux(a.valid === true.B && a.index > lastValidIndex, a, b)).index
  val thisValidIndex = Mux(valids(rightValidIndex), rightValidIndex, 
                          Mux(valids(leftValidIndex), leftValidIndex, 0.U))

  // search for the fully associative table
  val readIndex = Mux(io.update.fire(), idx(pageAddr), thisValidIndex)
  sTable.io.r.req.valid := io.update.fire() || valids(thisValidIndex)
  sTable.io.r.req.bits.setIdx := readIndex
  when(!io.update.fire) {
    lastValidIndex := thisValidIndex
  }

  // common logic for modify
  val readResult = Wire(new sTableEntry)
  readResult := sTable.io.r.resp.data(0)
  val delta = readResult.lastBlock - block_l
  val hitForIssue = readResult.valid
  val hitForUpdate = readResult.valid && readResult.tag === tag(pageAddr_l)

  // modify for update request
  val updated = Wire(new sTableEntry)
  val tempValid = WireDefault(false.B)
  val enwriteForUpdate = RegNext(io.update.fire()) && ((delta =/= 0.U && hitForUpdate) || !hitForUpdate)
  updated := readResult
  when(hitForUpdate) {
    val pState = readResult.state
    updated.lastBlock := block_l
    when(pState === s_searching) {
      val pageBlock = Cat(pageAddr_l, readResult.lastBlock)
      when(block_l > readResult.lastBlock && 
        ((samePage(pageBlock, pageBlock + dist_within_fb.asUInt) && block_l <= readResult.lastBlock + dist_within_fb.asUInt) ||
        (!samePage(pageBlock, pageBlock + dist_within_fb.asUInt) && block_l < 63.U))) {
        updated.state := s_forward
        updated.l2_home := block_l + dist_demand_l2home.asUInt
        updated.pendding := degree.asUInt
        updated.farBack := false.B
        tempValid := true.B
      }
      when(block_l < readResult.lastBlock && 
        ((samePage(pageBlock, pageBlock - dist_within_fb.asUInt) && block_l >= readResult.lastBlock - dist_within_fb.asUInt) ||
        (!samePage(pageBlock, pageBlock - dist_within_fb.asUInt) && block_l > 0.U))) {
        updated.state := s_backward
        updated.l2_home := block_l - dist_demand_l2home.asUInt
        updated.pendding := degree.asUInt
        updated.farBack := false.B
        tempValid := true.B
      }
    } .elsewhen(pState === s_forward || pState === s_backward) {
      when(inWindowA(readResult, block_l)) {
        when(readResult.farBack) {
          updated.state := s_searching
        } .otherwise {
          updated.farBack := true.B
        }
        val (h, s) = updateHome(pState, block_l)
        updated.l2_home := h
        updated.state := s
        when(s =/= s_done && readResult.pendding =/= 0.U) {
          tempValid := true.B
        }
      } .elsewhen(inWindowB(readResult, block_l)) {
        updated.pendding := readResult.pendding + degree.asUInt
        val (h, s) = updateHome(pState, block_l)
        updated.l2_home := h
        updated.state := s
        tempValid := s =/= s_done
      } .elsewhen(inWindowC(readResult, block_l)) {
        updated.pendding := readResult.pendding + degree.asUInt
        tempValid := true.B
      } .elsewhen(inWindowD(readResult, block_l)) {
        updated.pendding := readResult.pendding + degree.asUInt
        val (h, s) = updateHome(pState, block_l)
        updated.l2_home := h
        updated.state := s
        tempValid := s =/= s_done
      } .otherwise {
        assert(false.B)
      }
    }  .otherwise {
      // TODO: add done state with trigger
      // when()
      updated.state := s_searching
    }
  } .otherwise{
    updated.valid := true.B
    updated.tag := tag(pageAddr_l)
    updated.farBack := false.B
    updated.lastBlock := block_l
    when(block_l === 0.U || block_l === 63.U) {
      updated.state := Mux(block_l === 0.U, s_forward, s_backward)
      updated.l2_home := Mux(block_l === 0.U, dist_demand_l2home.asUInt, block_l - dist_demand_l2home.asUInt)
      updated.pendding := degree.asUInt
      tempValid := true.B
    } .otherwise {
      updated.state := s_searching
    }
  }
  when(hitForUpdate && (readResult.state === s_forward || readResult.state === s_backward) && !inWindowA(readResult, block_l)) {
    updated.farBack := false.B
  }
  when(io.update.fire()) {
    valids(idx(pageAddr_l)) := tempValid
  }

  // modify for issue request, update *state, l2_home, pendding
  val upIssue = Wire(new sTableEntry)
  upIssue := readResult
  val prefetchBlock = WireDefault(0.U((pageAddrBits + blkOffsetBits).W))
  val enwriteForIssue = WireDefault(false.B)
  when(hitForIssue && RegNext(sTable.io.r.req.fire()) && !io.update.fire()) {
    when((readResult.state === s_forward || readResult.state === s_backward) && readResult.pendding =/= 0.U) {
      enwriteForIssue := true.B
      val page = Cat(readResult.tag, RegNext(readIndex))
      val home = Mux(readResult.state === s_forward, readResult.l2_home + 1.U, readResult.l2_home - 1.U)  
      val done = home === 63.U || home === 0.U
      prefetchBlock := Cat(page, readResult.l2_home)
      when(done) {
        upIssue.state := s_done
      }
      upIssue.l2_home := home
      upIssue.pendding := readResult.pendding - 1.U
      when(readResult.pendding - 1.U === 0.U || done) {
        valids(lastValidIndex) := false.B
      }
    }
  }

  // write
  sTable.io.w.req.valid := enwriteForUpdate || enwriteForIssue
  sTable.io.w.req.bits.setIdx := RegNext(readIndex)
  sTable.io.w.req.bits.data(0) := Mux(enwriteForUpdate, updated, upIssue)

  io.issue.valid := enwriteForIssue
  io.issue.bits := prefetchBlock

  io.update.ready := sTable.io.r.req.ready

  // should sequentially test A,B,C,D
  def inWindowA(s: sTableEntry, demand: UInt): Bool = {
    val in = Wire(Bool())
    when(s.state === s_forward) {
      in := s.l2_home > dist_fb_home.asUInt && demand <= s.l2_home - dist_fb_home.asUInt
    } .otherwise {
      in := s.l2_home + dist_fb_home.asUInt > s.l2_home && demand >= s.l2_home + dist_fb_home.asUInt
    }
    in
  }
  def inWindowB(s: sTableEntry, demand: UInt): Bool = {
    val in = Wire(Bool())
    when(s.state === s_forward) {
      in := s.l2_home >= dist_tw_home.asUInt && demand <= s.l2_home - dist_tw_home.asUInt
    } .otherwise {
      in := s.l2_home + dist_tw_home.asUInt > s.l2_home && demand >= s.l2_home + dist_tw_home.asUInt
    }
    in
  }
  def inWindowC(s: sTableEntry, demand: UInt): Bool = {
    val in = Wire(Bool())
    when(s.state === s_forward) {
      in := demand <= s.l2_home + s.pendding
    } .otherwise {
      in := demand >= s.l2_home - s.pendding
    }
    in
  }
  def inWindowD(s: sTableEntry, demand: UInt): Bool = {
    val in = Wire(Bool())
    when(s.state === s_forward) {
      in := demand > s.l2_home + s.pendding
    } .otherwise {
      in := demand < s.l2_home - s.pendding
    }
    in
  }
  def updateHome(state: UInt, demand: UInt): (UInt, UInt) = {
    val newHome = Mux(state === s_forward, demand + dist_demand_l2home.asUInt, demand - dist_demand_l2home.asUInt)
    val done = (state === s_forward && demand > newHome) || (state === s_backward && demand < newHome)
    val nextState = Mux(done, s_done, state)
    (newHome, nextState)
  }
  def samePage(blockAddr1: UInt, blockAddr2: UInt): Bool = {
    val r = blockAddr1(pageAddrBits + blkOffsetBits - 1, blkOffsetBits) === 
              blockAddr2(pageAddrBits + blkOffsetBits - 1, blkOffsetBits)
    r
  }
}

class FilterMap(implicit p: Parameters) extends StreamerModule {
  val io = IO(new Bundle() {
    val req = Flipped(DecoupledIO(UInt((pageAddrBits + blkOffsetBits).W)))
    val resp = DecoupledIO(UInt((pageAddrBits + blkOffsetBits).W))
  })

  def idx(addr:      UInt) = addr(log2Up(fTableEntries) - 1, 0)
  def tag(addr:      UInt) = addr(pageAddrBits - 1, log2Up(fTableEntries))

  def fTableEntry() = new Bundle {
    val valid = Bool()
    val tag = UInt(sTagBits.W)
    val bitMap = Vec(64, Bool())
  }

  val fTable = Module(
    new SRAMTemplate(fTableEntry(), set = fTableEntries, way = 1, bypassWrite = true, shouldReset = true)
  )

  val oldAddr = io.req.bits
  val pageAddr = oldAddr(pageAddrBits + blkOffsetBits - 1, blkOffsetBits)
  val blkOffset = oldAddr(blkOffsetBits - 1, 0)

  //read fTable
  val hit = Wire(Bool())
  val readResult = Wire(fTableEntry())
  fTable.io.r.req.valid := io.req.valid
  fTable.io.r.req.bits.setIdx := idx(pageAddr)
  readResult := fTable.io.r.resp.data(0)
  hit := readResult.valid && tag(RegNext(pageAddr)) === readResult.tag

  val hitForMap = hit && readResult.bitMap(RegNext(blkOffset))

  io.resp.valid := RegNext(io.req.fire()) && !hitForMap
  io.resp.bits := RegNext(oldAddr)

  val newBitMap = Wire(Vec(64, Bool()))
  when(hit) {
    newBitMap := readResult.bitMap.zipWithIndex.map{ case (b, i) => Mux(i.asUInt === RegNext(blkOffset), true.B, b)}
  } .otherwise {
    newBitMap := readResult.bitMap.zipWithIndex.map{ case (b, i) => Mux(i.asUInt === RegNext(blkOffset), true.B, false.B)}
  }
  
  fTable.io.w.req.valid := RegNext(io.req.fire()) && !hitForMap
  fTable.io.w.req.bits.setIdx := idx(RegNext(pageAddr))
  fTable.io.w.req.bits.data(0).valid := true.B
  fTable.io.w.req.bits.data(0).tag := tag(RegNext(pageAddr))
  fTable.io.w.req.bits.data(0).bitMap := newBitMap

  io.req.ready := fTable.io.r.req.ready
}

class Streamer(implicit p: Parameters) extends StreamerModule {
  val io = IO(new Bundle() {
    val train = Flipped(DecoupledIO(new PrefetchTrain))
    val req = DecoupledIO(new PrefetchReq)
    val resp = Flipped(DecoupledIO(new PrefetchResp))
    val evict = Flipped(DecoupledIO(new PrefetchEvict))
  })
  val oldAddr = io.train.bits.addr //received request from L1 cache
  val pageAddr = getPPN(oldAddr)
  val blkOffset = oldAddr(pageOffsetBits - 1, offsetBits)

  val sTable = Module(new StreamerTable)
  val fTable = Module(new FilterMap)

  sTable.io.update.valid := io.train.fire()
  sTable.io.update.bits.pageAddr := pageAddr
  sTable.io.update.bits.blkOffset := blkOffset
  fTable.io.req <> sTable.io.issue

  val req = Reg(new PrefetchReq)
  val req_valid = RegInit(false.B)
  when(io.req.fire() && !fTable.io.resp.fire()) {
    req_valid := false.B
  }
  when(fTable.io.resp.fire()) {
    val newAddr = Cat(fTable.io.resp.bits, 0.U(offsetBits.W))
    req.tag := parseFullAddress(newAddr)._1
    req.set := parseFullAddress(newAddr)._2
    req.needT := true.B
    req.source := 0.U
    req.isBOP := false.B
    req_valid := true.B 
  }
  io.req.valid := req_valid
  io.req.bits := req

  io.resp.ready := true.B
  io.evict.ready := true.B
  io.train.ready := true.B
  fTable.io.resp.ready := true.B
}