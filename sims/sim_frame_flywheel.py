# Does the frame-marker flywheel survive the failure that killed the ring three
# times, WITHOUT changing anything in the healthy case?
#
# THE FAILURE. dante_packetizer's write pointer is `wr = frame_base`, and
# frame_base advances only on `samp_vld & first`, where `first` is the ch0
# marker the USB wrapper puts in bit 6 of each bridge word. If that marker stops
# arriving the ring is dead PERMANENTLY -- `started` is set only by `first` and
# do_write is gated on (started | first) -- while every upstream counter reads
# perfect. Observed 2026-08-12 and twice on 2026-08-13, always at ~3 hours,
# always needing a JTAG bitstream reload.
#
# THE TWO THINGS THAT MUST BOTH HOLD:
#   1. INERT WHEN HEALTHY. A marker every BLOCK_CH samples must never let the
#      flywheel fire. If it fires spuriously it advances the frame grid twice in
#      one frame and rotates every channel -- worse than the bug it fixes.
#   2. RECOVERS WHEN THE MARKER GOES. Within one frame the grid must advance
#      again, and the channel index must come from arrival position, because the
#      wrapper's tag is derived from the SAME marker and free-runs to 63 against
#      48 real channels once it is lost (usb_avb_subsystem.py: seq is
#      Signal(range(48)), six bits).
#
#   run:  python3 sims/sim_frame_flywheel.py

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from migen import *
from migen.sim import run_simulation
from dante_packetizer import DantePacketizer

BLOCK_CH = 48


class _MCR:
    def __init__(self): self.sample_strobe = Signal()


class _TSU:
    def __init__(self): self.timestamp = Signal(80)


def build():
    lo, hi, rdy = Signal(32), Signal(32), Signal()
    dut = DantePacketizer(_MCR(), _TSU(), usb_sample_lo=lo,
                          usb_sample_hi=hi, usb_readable=rdy)
    return dut, lo, hi, rdy


def feed(dut, lo, hi, rdy, n, marker_every, start_at=0):
    """Push n samples; assert the ch0 marker every `marker_every` (0 = never)."""
    log = []
    yield dut.enable.storage.eq(1)
    yield
    for i in range(n):
        idx = start_at + i
        is_first = (marker_every and (idx % marker_every == 0))
        ch = idx % BLOCK_CH
        yield hi.eq((ch & 0x3F) | ((1 << 6) if is_first else 0))
        yield lo.eq(0x11223300 | (ch & 0xFF))
        yield rdy.eq(1)
        yield
        log.append((( yield dut.dbg_frame_pos),
                    ( yield dut.dbg_holdover),
                    ( yield dut.dbg_chan_nr)))
    yield rdy.eq(0)
    yield
    return log


def main():
    fails = 0

    # --- 1. inert when healthy ---------------------------------------------
    res = {}
    def tb_healthy():
        log = yield from feed(dut1, lo1, hi1, rdy1, BLOCK_CH * 6, BLOCK_CH)
        res["healthy"] = log
        res["hc"] = yield dut1.holdover_count.status
    dut1, lo1, hi1, rdy1 = build()
    run_simulation(dut1, tb_healthy())
    hold_any = any(h for _, h, _ in res["healthy"])
    ok = (res["hc"] == 0) and not hold_any
    print(f"  {'PASS' if ok else 'FAIL'}  healthy: marker every {BLOCK_CH} -> "
          f"flywheel fired {res['hc']} times, holdover ever set: {hold_any}")
    if not ok:
        fails += 1

    # --- 2. recovers when the marker disappears ----------------------------
    res2 = {}
    def tb_lost():
        # 2 clean frames, then the marker never comes again.
        yield from feed(dut2, lo2, hi2, rdy2, BLOCK_CH * 2, BLOCK_CH)
        log = yield from feed(dut2, lo2, hi2, rdy2, BLOCK_CH * 4, 0,
                              start_at=BLOCK_CH * 2)
        res2["log"] = log
        res2["hc"] = yield dut2.holdover_count.status
    dut2, lo2, hi2, rdy2 = build()
    run_simulation(dut2, tb_lost())
    hc = res2["hc"]
    ok = hc >= 3                     # ~4 frames of holdover
    print(f"  {'PASS' if ok else 'FAIL'}  marker lost: flywheel synthesised "
          f"{hc} frame starts over 4 frames -- the grid keeps advancing")
    if not ok:
        fails += 1

    tail = res2["log"][BLOCK_CH:]    # after holdover has engaged
    hold_on = all(h for _, h, _ in tail)
    chans = sorted({c for _, h, c in tail if h})
    ok = hold_on and chans and max(chans) < BLOCK_CH
    print(f"  {'PASS' if ok else 'FAIL'}  in holdover the channel index stays "
          f"0..{max(chans) if chans else '?'} (< {BLOCK_CH}); the wrapper's own "
          f"tag would free-run to 63")
    if not ok:
        fails += 1

    # --- 3. a returning marker clears holdover ------------------------------
    res3 = {}
    def tb_return():
        yield from feed(dut3, lo3, hi3, rdy3, BLOCK_CH * 3, 0)
        log = yield from feed(dut3, lo3, hi3, rdy3, BLOCK_CH * 2, BLOCK_CH,
                              start_at=BLOCK_CH * 3)
        res3["log"] = log
    dut3, lo3, hi3, rdy3 = build()
    run_simulation(dut3, tb_return())
    after = [h for _, h, _ in res3["log"]]
    ok = after and after[-1] == 0
    print(f"  {'PASS' if ok else 'FAIL'}  marker returns -> holdover clears "
          f"(ends {after[-1] if after else '?'}), so a transient costs one frame")
    if not ok:
        fails += 1

    print(f"\n{'ALL CHECKS PASSED' if not fails else str(fails) + ' CHECK(S) FAILED'}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
