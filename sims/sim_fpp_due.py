# Why does any_due never fire on hardware?
#
# Symptom, seed 5 bitstream, six flows bound (one fpp=16, five fpp=60):
#
#   last_sec / last_ts  = 0    the emitted-timestamp latch NEVER fired
#   due_cnt[all]        = 0
#   packets             ~16/s against ~7000 expected
#
# The latch sits inside `If(strobe, If(any_due, ...))`, so last_ts == 0 is
# independent evidence that any_due never asserts -- it is not just the new
# counters misreading.
#
# Inspection of the RTL found nothing, so drive the REAL module: bind a context
# exactly the way firmware does, run strobes, and watch phase/fpp/due directly.
# This is the step that should have come before the first build.
#
#   run:  python3 sims/sim_fpp_due.py

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from migen import *

from dante_packetizer import DantePacketizer


class FakeTSU(Module):
    def __init__(self):
        self.seconds = Signal(48)
        self.nanoseconds = Signal(32)


class FakeMCR(Module):
    def __init__(self, div):
        self.sample_strobe = Signal()
        self.phase = Signal(32)
        cnt = Signal(max=div)
        self.sync += [
            self.sample_strobe.eq(0),
            If(cnt == (div - 1), cnt.eq(0), self.sample_strobe.eq(1)).Else(cnt.eq(cnt + 1)),
        ]


class Top(Module):
    def __init__(self, div=16, n_ctx=6):
        self.submodules.mcr = FakeMCR(div)
        self.submodules.tsu = FakeTSU()
        self.usb_lo = Signal(16)
        self.usb_hi = Signal(16)
        self.usb_rdy = Signal()
        self.submodules.p = DantePacketizer(
            mcr=self.mcr, tsu=self.tsu,
            usb_sample_lo=self.usb_lo, usb_sample_hi=self.usb_hi,
            usb_readable=self.usb_rdy, channels=8, samples_per_packet=16,
            fifo_depth=256, streams=6, n_ctx=n_ctx)


def bind(p, ctx, nslots, fpp_idx, phase):
    yield p.ctx_select.storage.eq(ctx)
    yield
    yield p.udp_dport.storage.eq(14000 + ctx)
    yield
    yield p.udp_dport.re.eq(1)
    yield
    yield p.udp_dport.re.eq(0)
    yield p.chmap_lo.storage.eq(0x83828180)
    yield p.chmap_hi.storage.eq(0)
    yield p.flow_phase.storage.eq(phase)
    yield p.flow_cfg.storage.eq((nslots & 0x0F) | ((fpp_idx & 7) << 4))
    yield
    yield p.flow_cfg.re.eq(1)
    yield
    yield p.flow_cfg.re.eq(0)
    yield


if __name__ == "__main__":
    # 128 sys cycles per media strobe, as sim_ring_accounting uses. At DIV=16
    # there are only 16 cycles between strobes to accept the 48 samples the host
    # delivers per sample-time, so the ring can never reach the prime floor and
    # NOTHING is ever built -- fifo_level topped out at 31 against a centre of
    # 64. That is a timescale artifact of the sim, not a fault in the design.
    # 1042 = 50e6/48000, the REAL ratio. At DIV=128 a sample is 128 cycles while
    # five fpp=60 packets take ~3855 cycles to build, so an fpp=16 context
    # (due every 2048 cycles at that scale) cannot possibly be served -- the
    # bench manufactures a throughput failure hardware does not have. Same trap
    # sim_ring_accounting.py documents.
    DIV = 1042
    dut = Top(div=DIV)
    p = dut.p
    fails = 0
    obs = {}

    BLOCK_CH = 48

    def tb():
        yield p.enable.storage.eq(1)
        yield
        # THE EXACT BENCH CONFIGURATION: one fpp=16 context (the AM2) alongside
        # five fpp=60 contexts (DVS). The earlier version bound only TWO
        # contexts and showed no half-rate, which is why it disagreed with
        # hardware -- the fault, if it is real, needs the full set.
        yield from bind(p, 0, 4, 1, 0)          # fpp=16
        for c in range(1, 6):
            yield from bind(p, c, 4, 4, 0)      # fpp=60
        yield p.ctx_select.storage.eq(0)
        # DRAIN THE OUTPUT. Nothing consumes p.source in this bench, so without
        # this source.ready stays low, the FSM stalls in STREAM forever and
        # pkt_count never moves -- which looks exactly like "the builder drops
        # everything" and is purely a missing sink.
        yield p.source.ready.eq(1)
        yield

        # USB INGRESS, PACED -- 48 samples per media strobe, as the real host
        # delivers. Without it the ring never primes, no packet is ever built,
        # and emit_cnt stays 0 regardless of whether the fix works. That is the
        # exact path the pending-latch fix touches, so it must be exercised.
        strobes = 0
        ch = 0
        owed = 0
        for c in range(DIV * 200):
            if (yield dut.mcr.sample_strobe):
                strobes += 1
                owed += BLOCK_CH
            yield dut.usb_rdy.eq(1 if owed > 0 else 0)
            yield dut.usb_hi.eq((ch & 0x3F) | (0x40 if ch == 0 else 0))
            yield dut.usb_lo.eq(((c & 0xFFFF) << 8) | ch)
            yield
            if owed > 0 and (yield p.usb_pop):
                ch = (ch + 1) % BLOCK_CH
                owed -= 1
        obs["strobes"] = strobes

        # Read what the module thinks each context is configured as.
        for c in range(6):
            yield p.ctx_select.storage.eq(c)
            yield
            yield
            obs[f"due_cnt{c}"] = (yield p.flow_due_cnt.status)
            obs[f"emit_cnt{c}"] = (yield p.flow_emit_cnt.status)
        obs["ts_now"] = (yield p.ts_now_sub.status)
        obs["fifo"] = (yield p.fifo_level.status)
        obs["pkts"] = (yield p.packet_count.status)
        obs["under"] = (yield p.underrun_count.status)

    run_simulation(dut, tb(), vcd_name="/tmp/sim_fpp_due.vcd")

    print(f"strobes driven: {obs['strobes']}")
    print(f"ts_now_sub    : {obs['ts_now']}   (should track the strobe count)")
    print(f"fifo_level    : {obs['fifo']}   pkt_count: {obs['pkts']}   underrun: {obs['under']}")
    for c, fpp in [(0, 16)] + [(i, 60) for i in range(1, 6)]:
        exp = obs["strobes"] // fpp
        got = obs[f"due_cnt{c}"]
        em = obs[f"emit_cnt{c}"]
        ok = got >= exp - 1 and em >= got * 0.9
        print(f"  {'PASS' if ok else 'FAIL'}  ctx{c} fpp={fpp}: due={got} "
              f"emit={em}  expected ~{exp}   "
              f"{'' if em >= got*0.9 else '<-- BUILDER STILL DROPPING'}")
        if not ok:
            fails += 1

    if fails:
        print("\nany_due is not firing in simulation either -- the fault is in the")
        print("RTL, not in the board. VCD at /tmp/sim_fpp_due.vcd")
    sys.exit(1 if fails else 0)
