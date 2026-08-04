# Migen simulation of the DantePacketizer RING ACCOUNTING, across context counts.
#
# WHY THIS EXISTS
#
# Raising the flow contexts 6 -> 12 (to serve 48 ch when receivers take only 4
# channels per flow) produced a working bitstream that then slammed the ring
# into both rails on hardware: overrun +2,864,963 in 100 s (28,650/s), level
# swinging 8..128 against a centre of 64. Audio counters had been clean at 6.
#
# The read/write accounting is not obvious from reading it:
#
#   writer:  wr = frame_base, advancing +channels (8) per USB 48-ch frame,
#            i.e. +8 per sample-time.
#   reader:  rd advances +64 ONCE PER BLOCK TRAVERSAL -- not per packet -- in
#            the `stream_idx == last` branch of both SKIP and the emit path.
#            64 = 8 sample-times x 8 channels-per-ring, so a block consumes
#            exactly 8 sample-times. Balanced against the writer at fpp=8.
#
# By that reading the CONTEXT COUNT should not enter the accounting at all,
# which is exactly why this needs simulating rather than more staring: the
# hardware disagreed with the analysis, so the analysis is wrong somewhere.
#
# WHAT THIS MEASURES
#
# Drive the real module with a synthetic 48-channel USB ingress at the nominal
# rate and a media strobe, bind N flows, and watch `level` (= wr - rd). A
# correct configuration holds level bounded; a broken one ramps.
#
# TIMESCALE CAVEAT -- READ BEFORE TRUSTING A FAIL.
#
# `div` is sys cycles per media strobe. Real hardware is 50e6/48000 = 1042, so a
# block (8 strobes at fpp=8) is 8336 cycles. This sim defaults to div=128
# (block = 1024) to keep run times sane, which is 8x LESS time to build the
# block's packets than hardware has.
#
# That compression manufactures throughput failures that hardware does not have.
# Measured here: 12 flows FAILS at div=128 (level 78..128, ring pinned full) and
# PASSES at div=512. On hardware, 12 packets x ~147 cycles = 1764 against a
# 8336-cycle block is 21% utilisation.
#
# So: a FAIL at low div means "the reader ran out of time", which is only a real
# finding if (packets_per_block x cycles_per_packet) approaches the real block
# period. Re-run the failing case at div=512 or higher before believing it.
#
#   run:  python3 sims/sim_ring_accounting.py

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from migen import *

from dante_packetizer import DantePacketizer


CHANNELS   = 8
SPP        = 16
BLOCK_CH   = 48


class FakeMCR(Module):
    """Media clock: one sample strobe every `div` sys cycles."""
    def __init__(self, div):
        self.sample_strobe = Signal()
        cnt = Signal(max=div + 1)
        self.sync += [
            self.sample_strobe.eq(0),
            If(cnt == (div - 1), cnt.eq(0), self.sample_strobe.eq(1)).Else(cnt.eq(cnt + 1)),
        ]


class FakeTSU(Module):
    def __init__(self):
        self.seconds     = Signal(48)
        self.nanoseconds = Signal(32)
        self.timestamp   = Signal(80)


class Bench(Module):
    def __init__(self, n_ctx, streams=6, div=128):
        # div=128: one media strobe per 128 sys cycles. Real hardware is
        # 50e6/48000 = 1042, but the RATIO that matters is strobe period vs the
        # time to build a block of packets (~150 cycles each). 128 keeps a block
        # of 8 strobes = 1024 cycles, the same relationship as hardware, while
        # running 8x fewer cycles per simulated second.
        self.submodules.mcr = FakeMCR(div=div)
        self.submodules.tsu = FakeTSU()
        self.usb_lo   = Signal(32)
        self.usb_hi   = Signal(8)
        self.usb_rdy  = Signal()
        kw = {}
        if n_ctx is not None:
            kw["n_ctx"] = n_ctx
        self.submodules.dut = DantePacketizer(
            mcr=self.mcr, tsu=self.tsu,
            usb_sample_lo=self.usb_lo, usb_sample_hi=self.usb_hi,
            usb_readable=self.usb_rdy,
            channels=CHANNELS, samples_per_packet=SPP,
            fifo_depth=256, streams=streams, **kw)
        # Sink the TX stream: always ready, so the MAC never backpressures and
        # the only thing under test is the ring accounting.
        self.comb += self.dut.source.ready.eq(1)


def bind_flow(dut, ctx, nslots, fpp, chans):
    """Write one flow context the way firmware does: fields first, dport LAST
    (the dport write strobe is what latches the context)."""
    yield dut.ctx_select.storage.eq(ctx)
    yield
    yield dut.dst_ip.storage.eq(0xC0A80001 + ctx)
    yield dut.ip_csum.storage.eq(0)
    yield dut.udp_sport.storage.eq(4321)
    cm = 0
    for i, c in enumerate(chans):
        cm |= (((c - 1) & 0x3F) | 0x80) << (8 * i)   # [5:0] 0-based ch, [7] valid
    # udp_dport.re latches the ip/udp fields
    yield dut.udp_dport.storage.eq(14000 + ctx)
    yield
    yield dut.udp_dport.re.eq(1)
    yield
    yield dut.udp_dport.re.eq(0)
    # dst_mac_lo.re latches the MAC
    yield dut.dst_mac_hi.storage.eq(0x0100)
    yield dut.dst_mac_lo.storage.eq(0x5E000001)
    yield
    yield dut.dst_mac_lo.re.eq(1)
    yield
    yield dut.dst_mac_lo.re.eq(0)
    # flow_cfg.re latches the channel map AND the slot/fpp config -- written LAST
    yield dut.chmap_lo.storage.eq(cm & 0xFFFFFFFF)
    yield dut.chmap_hi.storage.eq((cm >> 32) & 0xFFFFFFFF)
    yield dut.flow_cfg.storage.eq((nslots & 0x0F) | (0x10 if fpp == 16 else 0))
    yield
    yield dut.flow_cfg.re.eq(1)
    yield
    yield dut.flow_cfg.re.eq(0)
    yield


def run(n_ctx, flows, cycles=90000, div=128):
    """flows = list of (nslots, fpp, [channels])."""
    b = Bench(n_ctx=n_ctx, div=div)
    dut = b.dut
    levels = []

    def tb():
        for i, (nslots, fpp, chans) in enumerate(flows):
            yield from bind_flow(dut, i, nslots, fpp, chans)
        yield dut.enable.storage.eq(1)
        yield

        # USB INGRESS, PACED. The host delivers exactly BLOCK_CH samples per
        # sample-time; it does not free-run. Modelling it as always-ready floods
        # the ring and makes `level` meaningless -- the first version of this
        # sim did that and measured nothing. Budget 48 samples per media strobe.
        ch = 0
        owed = 0
        for c in range(cycles):
            if (yield b.mcr.sample_strobe):
                owed += BLOCK_CH
            yield b.usb_rdy.eq(1 if owed > 0 else 0)
            yield b.usb_hi.eq((ch & 0x3F) | (0x40 if ch == 0 else 0))
            yield b.usb_lo.eq(((c & 0xFFFF) << 8) | ch)
            yield
            if owed > 0 and (yield dut.usb_pop):
                ch = (ch + 1) % BLOCK_CH
                owed -= 1
            if c > cycles // 3 and c % 256 == 0:
                levels.append((yield dut.fifo_level.status))
        return

    run_simulation(b, tb())
    return levels


def verdict(name, levels):
    if not levels:
        return f"  {name}: no samples", False
    lo, hi = min(levels), max(levels)
    # A healthy ring is bounded and near centre; a broken one pins a rail.
    ok = lo > 4 and hi < 124
    drift = levels[-1] - levels[0]
    return (f"  {'PASS' if ok else 'FAIL'}  {name}: level {lo}..{hi} "
            f"(end-start {drift:+d})"), ok


if __name__ == "__main__":
    # The hardware case: an A16R taking four 4-channel flows plus an AM2 on two.
    FLOWS_5 = [(4, 8, [1, 2, 3, 4]), (4, 8, [13, 14, 15, 16]),
               (4, 8, [7, 6, 5, 8]), (4, 8, [9, 10, 11, 12]),
               (2, 16, [2, 1])]
    # 12 contexts fully loaded: 48 channels at 4 slots each.
    FLOWS_12 = [(4, 8, [4*i+1, 4*i+2, 4*i+3, 4*i+4]) for i in range(12)]
    # Sparse: one flow in a 12-context walk (most contexts skipped every block).
    FLOWS_1 = [(4, 8, [1, 2, 3, 4])]
    # Mixed fpp across many contexts -- fpp16 flows are due only on tick_hi, so
    # the walk emits a different number of packets on alternate blocks.
    FLOWS_MIX = [(4, 8 if i % 2 else 16, [4*i+1, 4*i+2, 4*i+3, 4*i+4]) for i in range(10)]

    cases = [
        ("n_ctx= 6, 5 flows  (shipping config)", 6,  FLOWS_5),
        ("n_ctx=12, 5 flows  (the hardware case)", 12, FLOWS_5),
        ("n_ctx=12, 12 flows (48ch target)",      12, FLOWS_12),
        ("n_ctx=12, 1 flow   (sparse walk)",      12, FLOWS_1),
        ("n_ctx=12, 10 mixed fpp",                12, FLOWS_MIX),
    ]
    fails = 0
    print("ring accounting vs flow-context count")
    for name, n, fl in cases:
        levels = run(n, fl)
        line, ok = verdict(name, levels)
        print(line)
        if not ok:
            fails += 1
    sys.exit(1 if fails else 0)
