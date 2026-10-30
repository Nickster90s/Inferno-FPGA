# Does the ring survive its own pointer wrap?
#
# IT DID NOT, AND IT KILLED THE AUDIO FOUR TIMES. rd advances by `channels` (8)
# per media strobe at 48 kHz = 384,000/s, and rd is 32 bits, so
#
#     2^32 / 384,000 = 11,185 s = 3.107 HOURS
#
# which is exactly when audio died on 2026-08-12, twice on 08-13 and again on
# 08-14, every time needing a JTAG bitstream reload.
#
# The mechanism: `level = wr - rd` was a PLAIN signed difference. wr leads rd by
# the ring level, so wr wraps first, and for the ~167 us until rd wraps too the
# difference reads -4,294,967,232 instead of +64. That un-primes the ring, and
# rd only advances under `strobe & primed` -- so rd freezes exactly when it was
# about to wrap, and never wraps. Permanent.
#
# NO SIMULATION IN THIS TREE EVER CROSSED THE WRAP. sim_ring_accounting and
# sim_ring_sample_rd both model occupancy over seconds, and the bug needs 2^32
# increments. That is why it shipped. This one drives the pointers straight at
# the boundary instead of waiting three hours to get there.
#
#   run:  python3 sims/sim_ring_wrap.py

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from migen import *
from migen.sim import run_simulation
from dante_packetizer import DantePacketizer

PTR_MOD = 1 << 32


class _MCR:
    def __init__(self): self.sample_strobe = Signal()


class _TSU:
    def __init__(self): self.timestamp = Signal(80)


def build():
    return DantePacketizer(_MCR(), _TSU(), usb_sample_lo=Signal(32),
                           usb_sample_hi=Signal(32), usb_readable=Signal())


def old_level(wr, rd):
    """What the shipped code computed: a plain widened difference."""
    return wr - rd


def main():
    dut = build()
    fails = 0
    got = {}

    def tb():
        # Park the design: strobe and usb_readable idle, so nothing in the
        # gateware writes rd or frame_base and the testbench can place the
        # pointers where it likes and read the real comb path. wr is COMB-DRIVEN
        # from frame_base, so drive that -- poking wr itself does nothing.
        yield dut.enable.storage.eq(1)
        yield
        out = []
        cases = []
        for true_level in (64, 8, 0):
            cases.append((true_level, true_level))            # nowhere near a wrap
            cases.append((true_level, 0))                     # wr JUST wrapped
        for true_level, wr in cases:
            rd = (wr - true_level) % PTR_MOD
            yield dut.dbg_frame_base.eq(wr)
            yield dut.dbg_rd.eq(rd)
            yield
            yield
            out.append((wr, rd, true_level,
                        (yield dut.dbg_level), (yield dut.block_level)))
        got["rows"] = out

    run_simulation(dut, tb())

    print(f"  {'wr':>12} {'rd':>12} {'true':>5} {'level':>8} {'blk':>4}  "
          f"{'OLD would be':>16}")
    for wr, rd, truth, lvl, blk in got["rows"]:
        # Migen reports a signed signal as a Python int already.
        ok = (lvl == truth)
        wrapped = "  <- ACROSS THE WRAP" if rd > PTR_MOD // 2 else ""
        print(f"  {wr:>12} {rd:>12} {truth:>5} {lvl:>8} {blk:>4}  "
              f"{old_level(wr, rd):>16}{wrapped}")
        if not ok:
            fails += 1

    print()
    # The load-bearing claim, stated as the failure it prevents.
    straddle = [r for r in got["rows"] if r[1] > PTR_MOD // 2]
    bad_old = [r for r in straddle if old_level(r[0], r[1]) != r[2]]
    print(f"  rows straddling the wrap: {len(straddle)}, of which the OLD "
          f"expression got {len(bad_old)} wrong")
    print(f"  worst OLD error: {min(old_level(r[0], r[1]) for r in straddle):,} "
          f"(the ring un-primes below {0}, and rd then freezes forever)")

    ok_all = fails == 0
    print(f"\n  {'PASS' if ok_all else 'FAIL'}  modular level is exact on both "
          f"sides of the 2^32 boundary")
    print(f"\n{'ALL CHECKS PASSED' if ok_all else str(fails) + ' CHECK(S) FAILED'}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
