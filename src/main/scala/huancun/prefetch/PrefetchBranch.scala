package huancun.prefetch

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import freechips.rocketchip.tilelink._
import huancun._
import utility.Pipeline

case class PrefetchBranchParams(n: Int = 32) extends PrefetchParameters {
  override val hasPrefetchBit: Boolean = true
  override val inflightEntries: Int = n
}

//Only used for hybrid spp and bop
class PrefetchBranch()(implicit p: Parameters) extends PrefetchModule {
  val io = IO(new Bundle() {
    val train = Flipped(DecoupledIO(new PrefetchTrain)) //from higher level cache
    val req = DecoupledIO(new PrefetchReq) //issue to next-level cache
    val resp = Flipped(DecoupledIO(new PrefetchResp)) //fill request from the next-level cache, using this to update filter
    val evict = Flipped(DecoupledIO(new PrefetchEvict))
  })

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
  io.req.valid := pf1.io.req.valid || pf2.io.req.valid
  io.req.bits := Mux(pf2.io.req.valid, pf2.io.req.bits, pf1.io.req.bits)

  io.train.ready := true.B
}