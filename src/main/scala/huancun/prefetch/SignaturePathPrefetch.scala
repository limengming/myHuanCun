// error when 000 signature occurs in pTable
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

case class SPPParameters(
  sTableEntries: Int = 1024,
  pTableEntries: Int = 4096,
  pTableDeltaEntries: Int = 4,
  pTableQueueEntries: Int = 8,
  strideIssues: Int = 7,
  signatureBits: Int = 12,
  fTableEntries: Int = 16,
  fTableQueueEntries: Int = 5
)
    extends PrefetchParameters {
  override val hasPrefetchBit:  Boolean = true
  override val inflightEntries: Int = 32
}

trait HasSPPParams extends HasHuanCunParameters {
  val sppParams = prefetchOpt.get.asInstanceOf[SPPParameters]

  val sTableEntries = sppParams.sTableEntries
  val pTableEntries = sppParams.pTableEntries
  val inflightEntries = sppParams.inflightEntries
  val pTableDeltaEntries = sppParams.pTableDeltaEntries
  val pTableQueueEntries = sppParams.pTableQueueEntries
  val strideIssues = sppParams.strideIssues
  val signatureBits = sppParams.signatureBits
  val fTableEntries = sppParams.fTableEntries
  val fTableQueueEntries = sppParams.fTableQueueEntries

  val pageAddrBits = fullAddressBits - pageOffsetBits
  val blkOffsetBits = pageOffsetBits - offsetBits
  val sTagBits = pageAddrBits - log2Up(sTableEntries)
  val fTagBits = pageAddrBits + blkOffsetBits - log2Up(fTableEntries)
}

abstract class SPPBundle(implicit val p: Parameters) extends Bundle with HasSPPParams
abstract class SPPModule(implicit val p: Parameters) extends Module with HasSPPParams

class DeltaEntry(implicit p: Parameters) extends SPPBundle {
  val delta = SInt((blkOffsetBits + 1).W)
  val cDelta = UInt(4.W)

  def apply(delta: SInt, cDelta: UInt) = {
    val entry = Wire(this)
    entry.delta := delta
    entry.cDelta := cDelta
    entry
  }
  def apply() = {
    val entry = Wire(this)
    entry.delta := 0.S
    entry.cDelta := 0.U
    entry
  }
}

class SignatureTableReq(implicit p: Parameters) extends SPPBundle {
  val pageAddr = UInt(pageAddrBits.W)
  val blkOffset = UInt(blkOffsetBits.W)
  val needT = Bool()
  val source = UInt(sourceIdBits.W)
}

class SignatureTableResp(implicit p: Parameters) extends SPPBundle {
  val signature = UInt(signatureBits.W)
  val delta = SInt((blkOffsetBits + 1).W)
  val block = UInt((pageAddrBits + blkOffsetBits).W)
  val needT = Bool()
  val source = UInt(sourceIdBits.W)
}

class PatternTableResp(implicit p: Parameters) extends SPPBundle {
  val deltaEntries = Vec(pTableDeltaEntries, new DeltaEntry())
  val block = UInt((pageAddrBits + blkOffsetBits).W)
  val needT = Bool()
  val source = UInt(sourceIdBits.W)
}

class FilterTableResp(implicit p: Parameters) extends SPPBundle {
  val prefetchBlock = UInt((pageAddrBits + blkOffsetBits).W)
  val needT = Bool()
  val source = UInt(sourceIdBits.W)
}

class SignatureTable(implicit p: Parameters) extends SPPModule {
  val io = IO(new Bundle {
    val req = Flipped(DecoupledIO(new SignatureTableReq))
    val resp = DecoupledIO(new SignatureTableResp) //output old signature and delta to write PT
  })
  def hash1(addr:    UInt) = addr(log2Up(sTableEntries) - 1, 0)
  def hash2(addr:    UInt) = addr(2 * log2Up(sTableEntries) - 1, log2Up(sTableEntries))
  def idx(addr:      UInt) = hash1(addr) ^ hash2(addr)
  def tag(addr:      UInt) = addr(pageAddrBits - 1, log2Up(sTableEntries))
  def sTableEntry() = new Bundle {
    val valid = Bool()
    val tag = UInt(sTagBits.W)
    val signature = UInt(signatureBits.W)
    val lastBlock = UInt(blkOffsetBits.W)
  }

  println(s"fullAddressBits: ${fullAddressBits}")
  println(s"pageOffsetBits: ${pageOffsetBits}")
  println(s"sTagBits: ${sTagBits}")
  
  val sTable = Module(
    new SRAMTemplate(sTableEntry(), set = sTableEntries, way = 1, bypassWrite = true, shouldReset = true)
  )

  val rAddr = io.req.bits.pageAddr
  val rData = Wire(sTableEntry())
  val lastAccessedPage = RegNext(io.req.bits.pageAddr)
  val lastAccessedBlock = RegNext(io.req.bits.blkOffset)

  sTable.io.r.req.valid := io.req.fire()
  sTable.io.r.req.bits.setIdx := idx(rAddr)
  rData := sTable.io.r.resp.data(0)

  val hit = rData.valid && rData.tag === tag(lastAccessedPage)
  val oldSignature = Mux(hit, rData.signature, 0.U)
  val newDelta = Mux(hit, lastAccessedBlock.asSInt - rData.lastBlock.asSInt, lastAccessedBlock.asSInt)

  sTable.io.w.req.valid := RegNext(sTable.io.r.req.fire()) && newDelta =/= 0.S
  sTable.io.w.req.bits.setIdx := idx(lastAccessedPage)
  sTable.io.w.req.bits.data(0).valid := true.B
  sTable.io.w.req.bits.data(0).tag := tag(lastAccessedPage)
  sTable.io.w.req.bits.data(0).signature := (oldSignature << 3) ^ newDelta.asUInt
  sTable.io.w.req.bits.data(0).lastBlock := lastAccessedBlock

  io.resp.valid := RegNext(sTable.io.r.req.fire()) && newDelta =/= 0.S
  io.resp.bits.signature := oldSignature
  io.resp.bits.delta := newDelta
  io.resp.bits.block := RegNext(Cat(io.req.bits.pageAddr, io.req.bits.blkOffset))
  io.resp.bits.needT := RegNext(io.req.bits.needT)
  io.resp.bits.source := RegNext(io.req.bits.source)

  io.req.ready := sTable.io.r.req.ready
}

class PatternTable(implicit p: Parameters) extends SPPModule {
  val io = IO(new Bundle {
    val req = Flipped(DecoupledIO(new SignatureTableResp))
    val resp = DecoupledIO(new PatternTableResp)
  })  

  def pTableEntry() = new Bundle {
    val valid = Bool()
    val deltaEntries = Vec(pTableDeltaEntries, new DeltaEntry())
    val count = UInt(4.W)
  }

  val pTable = Module(
    new SRAMTemplate(pTableEntry(), set = pTableEntries, way = 1, bypassWrite = true, shouldReset = true)
  )

  val enread = WireDefault(false.B)
  val req = Reg(new SignatureTableResp)
  val req_valid = RegInit(false.B)
  val readSignature = WireInit(0.U(signatureBits.W))
  val updatedSignature = Wire(UInt(signatureBits.W))
  updatedSignature := (req.signature << 3) ^ req.delta.asUInt
  when(io.req.fire()) {
    req := io.req.bits
    req_valid := true.B
    enread := true.B
    readSignature := io.req.bits.signature
  }
  io.req.ready := !req_valid

  def queueEntry() = new Bundle {
    val block = UInt((pageAddrBits + blkOffsetBits).W)
    val signature = UInt(signatureBits.W)
    val needT = Bool()
    val source = UInt(sourceIdBits.W)
    val prefetchMinCount = UInt(5.W)
    val delta = SInt((blkOffsetBits + 1).W)
  }
  val recursionQueue = Module(new ReplaceableQueueV2(queueEntry, pTableQueueEntries))
  val queue_req = recursionQueue.io.deq.bits
  val newEntry = WireInit(0.U.asTypeOf(queueEntry))
  recursionQueue.io.enq.bits := newEntry
  recursionQueue.io.deq.ready := false.B
  recursionQueue.io.enq.valid := false.B

  val lastDelta = RegNext(io.req.bits.delta)
  val lastSignature = RegNext(readSignature)
  val readResult = Wire(pTableEntry())
  val hit = WireDefault(false.B)

  val lookState = WireDefault(false.B)
  val lastLookState = RegNext(lookState)
  val enprefetch = WireDefault(false.B)
  val current = Reg(queueEntry)
  val look_delta = Mux(RegNext(req_valid), RegNext(req.delta), RegNext(queue_req.delta))

  //read pTable
  pTable.io.r.req.valid := enread || lookState
  pTable.io.r.req.bits.setIdx := readSignature
  readResult := pTable.io.r.resp.data(0)
  hit := readResult.valid
  //set output
  val maxEntry = readResult.deltaEntries.reduce((a, b) => Mux(a.cDelta >= b.cDelta, a, b))
  val delta_list = readResult.deltaEntries.map(x => 
            Mux(x.cDelta > current.prefetchMinCount,
            Mux(x.delta === look_delta, (new DeltaEntry).apply(x.delta, strideIssues.asUInt),
            (new DeltaEntry).apply(x.delta, 1.U)), (new DeltaEntry).apply()))
  val delta_list_checked = delta_list.map(x => 
            Mux((current.block.asSInt + x.delta).asUInt(pageAddrBits + blkOffsetBits - 1, blkOffsetBits)
            === current.block(pageAddrBits + blkOffsetBits - 1, blkOffsetBits), x, (new DeltaEntry).apply()))

  io.resp.bits.block := current.block
  io.resp.bits.deltaEntries := delta_list_checked
  io.resp.bits.needT := current.needT
  io.resp.bits.source := current.source
  io.resp.valid := enprefetch

  //modify table
  val deltaEntries = Wire(Vec(pTableDeltaEntries, new DeltaEntry()))
  val count = Wire(UInt(4.W))
  when(hit) {
    val exist = readResult.deltaEntries.map(_.delta === lastDelta).reduce(_ || _)
    when(exist) {
      val temp = readResult.deltaEntries.map(x =>
        Mux(x.delta === lastDelta, (new DeltaEntry).apply(lastDelta, x.cDelta + 1.U), x)) 
      //counter overflow
      when(readResult.count + 1.U === ((1.U << count.getWidth) - 1.U)) {
        // deltaEntries := temp.map(x => Mux(x.cDelta =/= 0.U, (new DeltaEntry).apply(x.delta, x.cDelta - 1.U), x))
        deltaEntries := temp.map(x => (new DeltaEntry).apply(x.delta, x.cDelta >> 1.U))
      } .otherwise {
        deltaEntries := temp
      }
      count := deltaEntries.map(_.cDelta).reduce(_ + _)
    } .otherwise {
      //to do replacement
      val smallest: SInt = readResult.deltaEntries.reduce((a, b) => {
        Mux(a.cDelta < b.cDelta, a, b)
      }).delta
      val indexToReplace : UInt = readResult.deltaEntries.indexWhere(a => a.delta === smallest)
      deltaEntries := VecInit.tabulate(readResult.deltaEntries.length) { i =>
        Mux((i.U === indexToReplace), (new DeltaEntry).apply(lastDelta, 1.U), 
        readResult.deltaEntries(i))
      }
      count := deltaEntries.map(_.cDelta).reduce(_ + _)
    }
    //to consider saturate here
  } .otherwise {
    deltaEntries := VecInit(Seq.fill(pTableDeltaEntries)((new DeltaEntry).apply(0.S, 0.U)))
    deltaEntries(0).delta := lastDelta
    deltaEntries(0).cDelta := 1.U
    count := 1.U
  }
  //write pTable
  pTable.io.w.req.valid := RegNext(io.req.fire() && pTable.io.r.req.fire())
  pTable.io.w.req.bits.setIdx := lastSignature
  pTable.io.w.req.bits.data(0).valid := true.B
  pTable.io.w.req.bits.data(0).deltaEntries := deltaEntries
  pTable.io.w.req.bits.data(0).count := count

  // lookahead read table
  when(!io.req.fire()) {
    when(req_valid) {
      readSignature := updatedSignature
      req_valid := false.B
      current.signature := updatedSignature
      current.block := req.block
      current.needT := req.needT
      current.source := req.source
      current.prefetchMinCount := 0.U
      lookState := true.B
    } .otherwise {
      recursionQueue.io.deq.ready := true.B
      when(recursionQueue.io.deq.fire()) {
        readSignature := queue_req.signature
        current.signature := queue_req.signature
        current.block := queue_req.block
        current.needT := queue_req.needT
        current.source := queue_req.source
        current.prefetchMinCount := queue_req.prefetchMinCount
        lookState := true.B
      }
    }
  }

  when(lastLookState && hit) {
    val issued = delta_list_checked.map(a => Mux(a.delta =/= 0.S, 1.U, 0.U)).reduce(_ +& _)
    when(issued =/= 0.U) {
      enprefetch := true.B
      val testOffset = (current.block.asSInt + maxEntry.delta).asUInt
      //same page?
      when(testOffset(pageAddrBits + blkOffsetBits - 1, blkOffsetBits) === 
        current.block(pageAddrBits + blkOffsetBits - 1, blkOffsetBits)
        && maxEntry.cDelta > current.prefetchMinCount) {
        newEntry.block := testOffset
        newEntry.signature := (lastSignature << 3) ^ maxEntry.delta.asUInt
        newEntry.needT := current.needT
        newEntry.source := current.source
        newEntry.prefetchMinCount := current.prefetchMinCount + 1.U
        newEntry.delta := maxEntry.delta
        recursionQueue.io.enq.valid := true.B
        recursionQueue.io.enq.bits := newEntry
      }
    }
  }
}

//Can add eviction notify or cycle counter for each entry
class FilterTable(implicit p: Parameters) extends SPPModule {
  val io = IO(new Bundle {
    val req = Flipped(DecoupledIO(new PatternTableResp))
    val evict = Flipped(DecoupledIO(new PrefetchEvict))
    val resp = DecoupledIO(new FilterTableResp)
  })

  def idx(addr:      UInt) = addr(log2Up(fTableEntries) - 1, 0)
  def tag(addr:      UInt) = addr(pageAddrBits + blkOffsetBits - 1, log2Up(fTableEntries)) 
  def fTableEntry() = new Bundle {
    val valid = Bool()
    val tag = UInt(fTagBits.W)
  }

  val inProcess = RegInit(false.B)
  val fTable = Module(
    new SRAMTemplate(fTableEntry(), set = fTableEntries, way = 1, bypassWrite = true, shouldReset = true)
  )

  val q = Module(new ReplaceableQueueV2(chiselTypeOf(io.req.bits), fTableQueueEntries))
  q.io.enq <> io.req //change logic to replace the tail entry

  val req = RegEnable(q.io.deq.bits, q.io.deq.fire())
  val req_deltas = Reg(Vec(pTableDeltaEntries, new DeltaEntry()))
  val normal_issue_finish = req_deltas.map(_.delta === 0.S).reduce(_ && _)
  val stride_issue_finish = WireDefault(true.B)
  when(q.io.deq.fire()) {
    req_deltas := q.io.deq.bits.deltaEntries
  }
  
  val rData = Wire(fTableEntry())
  val hit = WireDefault(false.B)
  val enread = WireDefault(false.B)
  val extract_delta = req_deltas.reduce((a, b) => Mux(a.delta =/= 0.S, a, b))
  val prefetchBlock = WireDefault(0.U((pageAddrBits + blkOffsetBits).W))

  val stride = RegInit(0.S((blkOffsetBits + 1).W))
  val strideCnt = RegInit(0.U(4.W))
  val strideBase = RegInit(0.U((pageAddrBits + blkOffsetBits).W))
  val enableStride = q.io.deq.bits.deltaEntries.map(x => Mux(x.cDelta > 1.U, true.B, false.B)).reduce(_ || _)
  when(q.io.deq.fire() && enableStride) {
    val i : UInt = q.io.deq.bits.deltaEntries.indexWhere(x => x.cDelta > 1.U)
    stride := q.io.deq.bits.deltaEntries(i).delta
    strideCnt := q.io.deq.bits.deltaEntries(i).cDelta
    strideBase := q.io.deq.bits.block
    req_deltas(i) := (new DeltaEntry).apply(0.S, 0.U)
  }

  fTable.io.r.req.valid := enread
  fTable.io.r.req.bits.setIdx := idx(prefetchBlock)
  rData := fTable.io.r.resp.data(0)
  hit := rData.valid && rData.tag === RegNext(tag(prefetchBlock))

  fTable.io.w.req.valid := !hit && RegNext(fTable.io.r.req.fire())
  fTable.io.w.req.bits.setIdx := idx(RegNext(prefetchBlock))
  fTable.io.w.req.bits.data(0).valid := true.B
  fTable.io.w.req.bits.data(0).tag := tag(RegNext(prefetchBlock))

  io.resp.valid := !hit && RegNext(fTable.io.r.req.fire())
  io.resp.bits.prefetchBlock := RegNext(prefetchBlock)
  io.resp.bits.source := RegNext(req.source)
  io.resp.bits.needT := RegNext(req.needT)

  when(inProcess) {
    when(!normal_issue_finish || strideCnt > 0.U) {
      when(strideCnt > 0.U) {
        val testOffset = (strideBase.asSInt + stride).asUInt
        when(testOffset(pageAddrBits + blkOffsetBits - 1, blkOffsetBits) === 
        req.block(pageAddrBits + blkOffsetBits - 1, blkOffsetBits)) {
          when(strideCnt > 1.U) {
            stride_issue_finish := false.B
          }
          enread := true.B
          prefetchBlock := testOffset
          strideBase := testOffset
          strideCnt := strideCnt - 1.U
        } .otherwise {
          strideCnt := 0.U
        }
      } .otherwise{
        enread := true.B
        prefetchBlock := (req.block.asSInt + extract_delta.delta).asUInt
        req_deltas := req_deltas.map(a => Mux(a.delta === extract_delta.delta, (new DeltaEntry).apply(0.S, 0.U), a))
      }
    } .otherwise {
      when(!q.io.deq.fire()) {
        inProcess := false.B
      }
    }
  } .otherwise {
    when(q.io.deq.fire()) {
      inProcess := true.B
    }
  }

  q.io.deq.ready := (!inProcess || normal_issue_finish) && stride_issue_finish
  io.evict.ready := true.B
}

class SignaturePathPrefetch(implicit p: Parameters) extends SPPModule {
  val io = IO(new Bundle() {
    val train = Flipped(DecoupledIO(new PrefetchTrain)) //from higher level cache
    val req = DecoupledIO(new PrefetchReq) //issue to next-level cache
    val resp = Flipped(DecoupledIO(new PrefetchResp)) //fill request from the next-level cache, using this to update filter
    val evict = Flipped(DecoupledIO(new PrefetchEvict))
  })

  val sTable = Module(new SignatureTable)
  val pTable = Module(new PatternTable)
  val fTable = Module(new FilterTable)

  val oldAddr = io.train.bits.addr //received request from L1 cache
  val pageAddr = getPPN(oldAddr)
  val blkOffset = oldAddr(pageOffsetBits - 1, offsetBits)

  io.train.ready := sTable.io.req.ready
  
  sTable.io.req.bits.pageAddr := pageAddr
  sTable.io.req.bits.blkOffset := blkOffset
  sTable.io.req.bits.needT := io.train.bits.needT
  sTable.io.req.bits.source := io.train.bits.source
  sTable.io.req.valid := io.train.fire()

  pTable.io.req <> sTable.io.resp //to detail
  pTable.io.resp <> fTable.io.req
  fTable.io.evict <> io.evict

  val req = Reg(new PrefetchReq)
  val req_valid = RegInit(false.B)
  fTable.io.resp.ready := true.B
  when(io.req.fire() && !fTable.io.resp.fire()) {
    req_valid := false.B
  }
  when(fTable.io.resp.fire()) {
    val newAddr = Cat(fTable.io.resp.bits.prefetchBlock, 0.U(offsetBits.W))
    req.tag := parseFullAddress(newAddr)._1
    req.set := parseFullAddress(newAddr)._2
    req.needT := fTable.io.resp.bits.needT
    req.source := fTable.io.resp.bits.source
    req_valid := true.B 
  }
  io.req.valid := req_valid
  io.req.bits := req
  io.req.bits.isBOP := false.B
  io.resp.ready := true.B

  val pf_trace = Wire(new Trace)
  pf_trace.paddr := Cat(req.tag, req.set, 0.U(offsetBits.W))
  pf_trace.carried := 0.U
  val table = ChiselDB.createTable("PrefetchTrace", new Trace)
  table.log(pf_trace, io.req.fire, "L2", clock, reset)

  val pf_train_trace = Wire(new Trace)
  pf_train_trace.paddr := Mux(io.train.fire, io.train.bits.addr, 
                              Cat(req.tag, req.set, 0.U(offsetBits.W)))
  pf_train_trace.carried := Mux(io.train.fire, 1.U, 0.U)
  val table2 = ChiselDB.createTable("PrefetchTrainTrace", new Trace)
  table2.log(pf_train_trace, io.req.fire || io.train.fire, "L2", clock, reset)

  XSPerfAccumulate(cacheParams, "recv_train", io.train.fire())
  XSPerfAccumulate(cacheParams, "recv_st", sTable.io.resp.fire())
  XSPerfAccumulate(cacheParams, "recv_pt", Mux(pTable.io.resp.fire(), 
      pTable.io.resp.bits.deltaEntries.map(a => Mux(a.delta =/= 0.S, 1.U, 0.U)).reduce(_ +& _), 0.U))
  XSPerfAccumulate(cacheParams, "recv_ft", fTable.io.resp.fire())
}