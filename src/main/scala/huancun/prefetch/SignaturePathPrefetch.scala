// error when 000 signature occurs in pTable
package huancun.prefetch

import huancun.utils.SRAMTemplate
import huancun.utils.ReplaceableQueue
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
  pTableQueueEntries: Int = 5,
  signatureBits: Int = 12,
  fTableEntries: Int = 128,
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
  val signatureBits = sppParams.signatureBits
  val pTableQueueEntries = sppParams.pTableQueueEntries
  val fTableEntries = sppParams.fTableEntries
  val fTableQueueEntries = sppParams.fTableQueueEntries

  val pageAddrBits = fullAddressBits - pageOffsetBits
  val blkOffsetBits = pageOffsetBits - offsetBits
  val sTagBits = pageAddrBits - log2Up(sTableEntries)
  val fTagBits = pageAddrBits + blkOffsetBits - log2Up(fTableEntries)
}

abstract class SPPBundle(implicit val p: Parameters) extends Bundle with HasSPPParams
abstract class SPPModule(implicit val p: Parameters) extends Module with HasSPPParams

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
  val deltas = Vec(pTableDeltaEntries, SInt((blkOffsetBits + 1).W))
  val block = UInt((pageAddrBits + blkOffsetBits).W)
  val needT = Bool()
  val source = UInt(sourceIdBits.W)
}

class FilterTableResp(implicit p: Parameters) extends SPPBundle {
  val prefetchBlock = UInt((pageAddrBits + blkOffsetBits).W)
  val needT = Bool()
  val source = UInt(sourceIdBits.W)
}

class DeltaEntry(implicit p: Parameters) extends SPPBundle {
  val delta = SInt((blkOffsetBits + 1).W)
  val cDelta = UInt(4.W)

  def apply(delta: SInt, cDelta: UInt) = {
    val entry = Wire(this)
    entry.delta := delta
    entry.cDelta := cDelta
    entry
  }
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
    new SRAMTemplate(sTableEntry(), set = sTableEntries, way = 1, shouldReset = true, singlePort = true)
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

  io.req.ready := !sTable.io.w.req.fire()
}

class PatternTable(implicit p: Parameters) extends SPPModule {
  val io = IO(new Bundle {
    val req = Flipped(DecoupledIO(new SignatureTableResp))
    val resp = DecoupledIO(new PatternTableResp)
  })  

  def pTableEntry() = new Bundle {
    val valid = Bool()
    //val deltaEntries = VecInit(Seq.fill(pTableDeltaEntries)((new DeltaEntry).apply(0.S, 0.U)))
    val deltaEntries = Vec(pTableDeltaEntries, new DeltaEntry())
    val count = UInt(4.W)
  }

  val pTable = Module(
    new SRAMTemplate(pTableEntry(), set = pTableEntries, way = 1, bypassWrite = true, shouldReset = true)
  )

  val q = Module(new ReplaceableQueue(chiselTypeOf(io.req.bits), pTableQueueEntries))
  q.io.enq <> io.req
  val req = q.io.deq.bits

  val s_idle :: s_lookahead0 :: s_lookahead :: Nil = Enum(3)
  val state = RegInit(s_idle)
  val readResult = Wire(pTableEntry())
  val readSignature = WireDefault(0.U(signatureBits.W)) //to differentiate the result from io or lookahead, set based on state
  val readDelta = WireDefault(0.S((blkOffsetBits + 1).W))
  val lastSignature = Wire(UInt(signatureBits.W))
  val lastDelta = Wire(SInt((blkOffsetBits + 1).W))
  val hit = WireDefault(false.B)
  val enread = WireDefault(false.B)
  val enprefetch = WireDefault(false.B)
  val enwrite = RegNext(q.io.deq.fire() && pTable.io.r.req.fire()) //we only modify-write on demand requests
  val current = Reg(new SignatureTableResp) // RegInit(0.U.asTypeOf(new PatternTableResp))
  val lookCount = RegInit(0.U(5.W))

  //read pTable
  pTable.io.r.req.valid := enread
  pTable.io.r.req.bits.setIdx := readSignature
  readResult := pTable.io.r.resp.data(0)
  hit := readResult.valid
  lastSignature := RegNext(readSignature)
  lastDelta := RegNext(readDelta)
  //set output
  val maxEntry = readResult.deltaEntries.reduce((a, b) => Mux(a.cDelta >= b.cDelta, a, b))
  val delta_list = readResult.deltaEntries.map(x => Mux(x.cDelta > lookCount, x.delta, 0.S))
  val delta_list_checked = delta_list.map(x => 
            Mux((current.block.asSInt + x).asUInt(pageAddrBits + blkOffsetBits - 1, blkOffsetBits)
            === current.block(pageAddrBits + blkOffsetBits - 1, blkOffsetBits), x, 0.S))

  io.resp.bits.block := current.block
  io.resp.bits.deltas := delta_list_checked
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
  pTable.io.w.req.valid := enwrite
  pTable.io.w.req.bits.setIdx := lastSignature
  pTable.io.w.req.bits.data(0).valid := true.B
  pTable.io.w.req.bits.data(0).deltaEntries := deltaEntries
  pTable.io.w.req.bits.data(0).count := count
  
  //FSM
  switch(state) {
    is(s_idle) {
      when(q.io.deq.fire()) {
        readSignature := req.signature
        readDelta := req.delta
        state := s_lookahead0
        current := req
        enread := true.B
      }
    }
    is(s_lookahead0) {
      enread := true.B
      readSignature := (lastSignature << 3) ^ lastDelta.asUInt
      state := s_lookahead
    }
    is(s_lookahead) {
      when(hit) {
        val issued = delta_list_checked.map(a => Mux(a =/= 0.S, 1.U, 0.U)).reduce(_ +& _)
        when(issued =/= 0.U) {
          enprefetch := true.B
          val testOffset = (current.block.asSInt + maxEntry.delta).asUInt
          //same page?
          when(testOffset(pageAddrBits + blkOffsetBits - 1, blkOffsetBits) === 
            current.block(pageAddrBits + blkOffsetBits - 1, blkOffsetBits)
            && maxEntry.cDelta > lookCount) {
            lookCount := lookCount + 1.U
            readSignature := (lastSignature << 3) ^ maxEntry.delta.asUInt
            current.block := testOffset
            enread := true.B
          } .otherwise {
            lookCount := 0.U
            state := s_idle
          }
        }.otherwise {
          lookCount := 0.U
          state := s_idle
        } 
      } .otherwise {
        lookCount := 0.U
        state := s_idle
      }
    }
  }

  q.io.deq.ready := state === s_idle
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

  val q = Module(new ReplaceableQueue(chiselTypeOf(io.req.bits), fTableQueueEntries))
  q.io.enq <> io.req //change logic to replace the tail entry

  val req = RegEnable(q.io.deq.bits, q.io.deq.fire())
  val req_deltas = Reg(Vec(pTableDeltaEntries, SInt((blkOffsetBits + 1).W)))
  val issue_finish = req_deltas.map(_ === 0.S).reduce(_ && _)
  q.io.deq.ready := !inProcess || issue_finish
  when(q.io.deq.fire()) {
    req_deltas := q.io.deq.bits.deltas
  }
  

  val rData = Wire(fTableEntry())
  val hit = WireDefault(false.B)
  val enread = WireDefault(false.B)
  val extract_delta = req_deltas.reduce((a, b) => Mux(a =/= 0.S, a, b))
  val prefetchBlock = (req.block.asSInt + extract_delta).asUInt

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
    when(!issue_finish) {
      enread := true.B
      req_deltas := req_deltas.map(a => Mux(a === extract_delta, 0.S, a))
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

  // might be lack of prefetch requests
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
  fTable.io.resp.ready := !req_valid
  when(io.req.fire()) {
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
      pTable.io.resp.bits.deltas.map(a => Mux(a =/= 0.S, 1.U, 0.U)).reduce(_ +& _), 0.U))
  XSPerfAccumulate(cacheParams, "recv_ft", fTable.io.resp.fire())
}