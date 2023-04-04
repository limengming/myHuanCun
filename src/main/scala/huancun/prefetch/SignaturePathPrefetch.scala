package huancun.prefetch

import huancun.utils.{ChiselDB, SRAMTemplate}
import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import huancun.HasHuanCunParameters

case class SPPParameters(
  sTableEntries: Int = 256,
  pTableEntries: Int = 512,
  pTableDeltaEntries: Int = 4,
  signatureBits: Int = 12
)
    extends PrefetchParameters {
  override val hasPrefetchBit:  Boolean = true
  override val inflightEntries: Int = 16
}

trait HasSPPParams extends HasHuanCunParameters {
  val sppParams = prefetchOpt.get.asInstanceOf[SPPParameters]

  val sTableEntries = sppParams.sTableEntries
  val pTableEntries = sppParams.pTableEntries
  val inflightEntries = sppParams.inflightEntries
  val pTableDeltaEntries = sppParams.pTableDeltaEntries
  val signatureBits = sppParams.signatureBits

  val pageAddrBits = fullAddressBits - pageOffsetBits
  val blkOffsetBits = pageOffsetBits - offsetBits
  val sTagBits = pageAddrBits - log2Up(sTableEntries)
  val pTagBits = signatureBits - log2Up(pTableEntries)
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
  val pageOffset = UInt((pageAddrBits + blkOffsetBits).W)
  val needT = Bool()
  val source = UInt(sourceIdBits.W)
}

class PatternTableResp(implicit p: Parameters) extends SPPBundle {
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
  def idx(addr:      UInt) = addr(log2Up(sTableEntries) - 1, 0)
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

  sTable.io.r.req.valid := io.req.fire()
  sTable.io.r.req.bits.setIdx := idx(rAddr)
  rData := sTable.io.r.resp.data(0)

  val hit = rData.valid && rData.tag === RegNext(tag(rAddr))
  val oldSignature = Mux(hit, rData.signature, 0.U)
  val newDelta = Mux(hit, io.req.bits.blkOffset.asSInt - rData.lastBlock.asSInt, io.req.bits.blkOffset.asSInt)

  sTable.io.w.req.valid := RegNext(sTable.io.r.req.fire())
  sTable.io.w.req.bits.setIdx := idx(rAddr)
  sTable.io.w.req.bits.data(0).valid := true.B
  sTable.io.w.req.bits.data(0).tag := tag(rAddr)
  sTable.io.w.req.bits.data(0).signature := (oldSignature << 3) ^ newDelta.asUInt
  sTable.io.w.req.bits.data(0).lastBlock := io.req.bits.blkOffset

  io.resp.valid := RegNext(sTable.io.r.req.fire())
  io.resp.bits.signature := oldSignature
  io.resp.bits.delta := newDelta
  io.resp.bits.pageOffset := RegNext(Cat(io.req.bits.pageAddr, io.req.bits.blkOffset))
  io.resp.bits.needT := RegNext(io.req.bits.needT)
  io.resp.bits.source := RegNext(io.req.bits.source)

  io.req.ready := !sTable.io.w.req.fire()
}

class PatternTable(implicit p: Parameters) extends SPPModule {
  val io = IO(new Bundle {
    val req = Flipped(DecoupledIO(new SignatureTableResp))
    val resp = DecoupledIO(new PatternTableResp)
  })
  def idx(addr:      UInt) = addr(log2Up(pTableEntries) - 1, 0)
  def tag(addr:      UInt) = addr(11, log2Up(pTableEntries))  

  def pTableEntry() = new Bundle {
    val valid = Bool()
    val tag = UInt(pTagBits.W)
    //val deltaEntries = VecInit(Seq.fill(pTableDeltaEntries)((new DeltaEntry).apply(0.S, 0.U)))
    val deltaEntries = Vec(pTableDeltaEntries, new DeltaEntry())
    val count = UInt(4.W)
  }

  val pTable = Module(
    new SRAMTemplate(pTableEntry(), set = pTableEntries, way = 1, bypassWrite = true)
  )

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
  val enwrite = RegNext(io.req.fire() && pTable.io.r.req.fire()) //we only modify-write on demand requests
  val current = Reg(new PatternTableResp) // RegInit(0.U.asTypeOf(new PatternTableResp))

  //read pTable
  pTable.io.r.req.valid := enread
  pTable.io.r.req.bits.setIdx := idx(readSignature)
  readResult := pTable.io.r.resp.data(0)
  hit := readResult.valid && readResult.tag === RegNext(tag(readSignature))
  lastSignature := RegNext(readSignature)
  lastDelta := RegNext(readDelta)
  //set output
  val maxEntry = readResult.deltaEntries.reduce((a, b) => Mux(a.cDelta >= b.cDelta, a, b))
  io.resp.bits.prefetchBlock := (current.prefetchBlock.asSInt + maxEntry.delta).asUInt
  io.resp.bits.needT := current.needT
  io.resp.bits.source := current.source
  io.resp.valid := enprefetch && 
    ((maxEntry.cDelta << 2.U) >= ((readResult.count << 1.U).asTypeOf(UInt(6.W)) + readResult.count))

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
      val smallest = readResult.deltaEntries.reduce((a, b) => {
        Mux(a.cDelta <= b.cDelta, a, b)
      })
      deltaEntries := readResult.deltaEntries.map(x =>
        Mux(x.delta === smallest.delta, (new DeltaEntry).apply(lastDelta, 1.U), x))
      count := readResult.count - smallest.cDelta + 1.U
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
  pTable.io.w.req.bits.setIdx := idx(lastSignature)
  pTable.io.w.req.bits.data(0).valid := true.B
  pTable.io.w.req.bits.data(0).tag := tag(lastSignature)
  pTable.io.w.req.bits.data(0).deltaEntries := deltaEntries
  pTable.io.w.req.bits.data(0).count := count
  
  //FSM
  switch(state) {
    is(s_idle) {
      when(io.req.fire()) {
        readSignature := io.req.bits.signature
        readDelta := io.req.bits.delta
        state := s_lookahead0
        current.prefetchBlock := io.req.bits.pageOffset
        current.needT := io.req.bits.needT
        current.source := io.req.bits.source
        enread := true.B
      }
    }
    is(s_lookahead0) {
      enread := true.B
      when(io.req.fire()) {
        readSignature := io.req.bits.signature
        readDelta := io.req.bits.delta
        current.prefetchBlock := io.req.bits.pageOffset
        current.needT := io.req.bits.needT
        current.source := io.req.bits.source
      } .otherwise {
        readSignature := (lastSignature << 3) ^ lastDelta.asUInt
        state := s_lookahead
      }
    }
    is(s_lookahead) {
      when(io.req.fire()) {
        readSignature := io.req.bits.signature
        readDelta := io.req.bits.delta
        state := s_lookahead0
        current.prefetchBlock := io.req.bits.pageOffset
        current.needT := io.req.bits.needT
        current.source := io.req.bits.source
        enread := true.B
      }
      when(!io.req.fire() && hit) {
        val testOffset = (current.prefetchBlock.asSInt + maxEntry.delta).asUInt
        //same page?
        when(testOffset(pageAddrBits + blkOffsetBits - 1, blkOffsetBits) === 
          current.prefetchBlock(pageAddrBits + blkOffsetBits - 1, blkOffsetBits)) {
          readSignature := (lastSignature << 3) ^ maxEntry.delta.asUInt
          current.prefetchBlock := testOffset
          enread := true.B
          enprefetch := true.B
        } .otherwise {
          state := s_idle
        }
      }
      when(!io.req.fire() && !hit) {
        state := s_idle
      }
    }
  }

  io.req.ready := !pTable.io.w.req.fire()
}

class SignaturePathPrefetch(implicit p: Parameters) extends SPPModule {
  val io = IO(new Bundle() {
    val train = Flipped(DecoupledIO(new PrefetchTrain)) //from higher level cache
    val req = DecoupledIO(new PrefetchReq) //issue to next-level cache
    val resp = Flipped(DecoupledIO(new PrefetchResp)) //fill request from the next-level cache, using this to update filter
  })

  val sTable = Module(new SignatureTable)
  val pTable = Module(new PatternTable)

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
  pTable.io.resp.ready := true.B

  val req = Reg(new PrefetchReq)
  val req_valid = RegInit(false.B)
  when(io.req.fire()) {
    req_valid := false.B
  }
  when(pTable.io.resp.valid) {
    val newAddr = Cat(pTable.io.resp.bits.prefetchBlock, 0.U(offsetBits.W))
    req.tag := parseFullAddress(newAddr)._1
    req.set := parseFullAddress(newAddr)._2
    req.needT := pTable.io.resp.bits.needT
    req.source := pTable.io.resp.bits.source
    req_valid := true.B 
  }
  io.req.valid := req_valid
  io.req.bits := req
  io.req.bits.isBOP := false.B
  io.resp.ready := true.B

  val pf_trace = Wire(new Trace)
  pf_trace.paddr := Cat(req.tag, req.set, 0.U(offsetBits.W))
  pf_trace.pc := 0.U
  val table = ChiselDB.createTable("PrefetchTrace", new Trace)
  table.log(pf_trace, io.req.fire, "L2", clock, reset)
}