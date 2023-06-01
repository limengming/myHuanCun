package huancun.prefetch

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import freechips.rocketchip.tilelink._
import huancun._
import huancun.utils.SRAMTemplate
import utility.Pipeline

case class PrefetchBranchParams(
  fTableEntries: Int = 32
)
    extends PrefetchParameters {
  override val hasPrefetchBit:  Boolean = true
  override val inflightEntries: Int = 32
}

trait HasPrefetchBranchParams extends HasHuanCunParameters {
  val prefetchBranchParams = prefetchOpt.get.asInstanceOf[PrefetchBranchParams]

  val pageAddrBits = fullAddressBits - pageOffsetBits
  val blkOffsetBits = pageOffsetBits - offsetBits

  val fTableEntries = prefetchBranchParams.fTableEntries
  val fTagBits = pageAddrBits - log2Up(fTableEntries)
}

abstract class PrefetchBranchModule(implicit val p: Parameters) extends Module with HasPrefetchBranchParams
abstract class PrefetchBranchBundle(implicit val p: Parameters) extends Bundle with HasPrefetchBranchParams

class Filter(implicit p: Parameters) extends PrefetchBranchModule {
  val io = IO(new Bundle() {
    val req = Flipped(DecoupledIO(new PrefetchReq))
    val resp = DecoupledIO(new PrefetchReq)
  })

  def idx(addr:      UInt) = addr(log2Up(fTableEntries) - 1, 0)
  def tag(addr:      UInt) = addr(pageAddrBits - 1, log2Up(fTableEntries))

  def fTableEntry() = new Bundle {
    val valid = Bool()
    val tag = UInt(fTagBits.W)
    val bitMap = Vec(64, Bool())
  }

  val fTable = Module(
    new SRAMTemplate(fTableEntry(), set = fTableEntries, way = 1, bypassWrite = true, shouldReset = true)
  )

  val oldAddr = io.req.bits.addr
  val pageAddr = oldAddr(fullAddressBits - 1, pageOffsetBits)
  val blkOffset = oldAddr(pageOffsetBits - 1, offsetBits)

  //read fTable
  val hit = Wire(Bool())
  val readResult = Wire(fTableEntry())
  fTable.io.r.req.valid := io.req.valid
  fTable.io.r.req.bits.setIdx := idx(pageAddr)
  readResult := fTable.io.r.resp.data(0)
  hit := readResult.valid && tag(RegNext(pageAddr)) === readResult.tag

  val hitForMap = hit && readResult.bitMap(RegNext(blkOffset))

  io.resp.valid := RegNext(io.req.fire()) && !hitForMap
  io.resp.bits := RegNext(io.req.bits)

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

//Only used for hybrid spp and bop
class PrefetchBranch()(implicit p: Parameters) extends PrefetchBranchModule {
  val io = IO(new Bundle() {
    val train = Flipped(DecoupledIO(new PrefetchTrain)) //from higher level cache
    val req = DecoupledIO(new PrefetchReq) //issue to next-level cache
    val resp = Flipped(DecoupledIO(new PrefetchResp)) //fill request from the next-level cache, using this to update filter
    val evict = Flipped(DecoupledIO(new PrefetchEvict))
  })

  val fTable = Module(new Filter)

  val pf1 = Module(new SignaturePathPrefetch()(p.alterPartial({
        case HCCacheParamsKey => p(HCCacheParamsKey).copy(prefetch = Some(SPPParameters()))
  })))
  val pf2 = Module(new BestOffsetPrefetch()(p.alterPartial({
        case HCCacheParamsKey => p(HCCacheParamsKey).copy(prefetch = Some(BOPParameters()))
  })))

  pf1.io.train.valid := io.train.valid
  pf1.io.train.bits := io.train.bits

  val train_for_pf2 = Reg(new PrefetchTrain)
  val train_for_pf2_valid = RegInit(false.B)
  
  when(io.train.valid && !pf2.io.train.ready) {
    train_for_pf2 := io.train.bits
    train_for_pf2_valid := true.B
  }
  pf2.io.train.valid := io.train.valid || train_for_pf2_valid
  pf2.io.train.bits := Mux(io.train.valid, io.train.bits, train_for_pf2)
  when(pf2.io.train.fire() && !io.train.valid) {
    train_for_pf2_valid := false.B
  }

  pf2.io.resp <> io.resp
  pf1.io.evict <> io.evict
  pf1.io.resp.bits.tag := 0.U
  pf1.io.resp.bits.set := 0.U
  pf1.io.resp.valid := false.B

  pf1.io.req.ready := !pf2.io.req.valid
  pf2.io.req.ready := true.B

  fTable.io.req.valid := pf1.io.req.valid || pf2.io.req.valid
  fTable.io.req.bits := Mux(pf2.io.req.valid, pf2.io.req.bits, pf1.io.req.bits)

  io.req <> fTable.io.resp

  io.train.ready := true.B
}