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
# 2026-11-06: THE POINTERS ARE 48 BITS NOW, so 2^32 is no longer a boundary at
# all -- it is an ordinary count. This sim therefore checks TWO things: that the
# old 3.107-hour cliff is gone (nothing special happens at 2^32), and that the
# real boundary, wherever PTR_BITS puts it, is still exact. The boundary is read
# from the DUT rather than hardcoded, so widening the pointers again cannot
# silently stop testing anything.
#
# It also checks the health instrumentation added with the widening, because a
# counter nobody has ever seen increment is not evidence of anything.
#
#   run:  python3 sims/sim_ring_wrap.py

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from migen import *
from migen.sim import run_simulation
from dante_packetizer import DantePacketizer

OLD_PTR_MOD = 1 << 32     # the width that wedged the audio four times


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


def to_signed(value, bits):
    return value - (1 << bits) if value >> (bits - 1) else value


def health(dut):
    """Do the ring-health CSRs actually report an inverted ring?

    The 2026-08 wedges were invisible because block_level clamps a negative
    level to 0. These four CSRs exist so that can never happen again -- which
    is only true if they move when the ring really does invert.
    """
    out = {}

    def tb():
        mod = 1 << len(dut.dbg_rd)
        # THE LEVELS HAVE TO COME FROM THE RING, NOT FROM A ROUND NUMBER. The
        # modulus is 4*SRING_DEPTH, so only 0..2*SRING_DEPTH-1 reads positive;
        # a first draft of this test used +1024 on a 512-sample ring and got
        # -1024 back, then blamed the counters. Real occupancy cannot exceed
        # SRING_DEPTH, so that value was never reachable -- the test was wrong,
        # not the design, and the arithmetic said so immediately.
        depth  = 1 << max(1, (dut.fifo_depth * 8 - 1).bit_length())
        center = depth // 2                     # where the ring primes
        base   = 4 * depth                      # anywhere, away from 0
        yield dut.enable.storage.eq(1)
        yield
        # Quiet start: a healthy primed ring must leave every counter alone.
        yield dut.dbg_frame_base.eq(base)
        yield dut.dbg_rd.eq(base - center)
        for _ in range(4):
            yield
        out["primed"]      = (yield dut.block_level)
        out["clean_uf"]    = (yield dut.ring_underflow.status)
        out["clean_min"]   = to_signed((yield dut.ring_level_min.status), 32)
        # Now invert it: the consumer ahead of the producer, which is the thing
        # that cannot happen -- and did, for three hours at a time.
        yield dut.dbg_rd.eq((base + 64) % mod)  # level -64
        for _ in range(4):
            yield
        out["raw"]      = to_signed((yield dut.ring_level_raw.status), 32)
        out["blk"]      = (yield dut.block_level)
        out["uf"]       = (yield dut.ring_underflow.status)
        out["min"]      = to_signed((yield dut.ring_level_min.status), 32)
        out["unprime"]  = (yield dut.ring_unprime.status)
        # Recover, then invert a second time: the counter must count EPISODES.
        yield dut.dbg_rd.eq(base - center)
        for _ in range(4):
            yield
        yield dut.dbg_rd.eq((base + 8) % mod)
        for _ in range(4):
            yield
        out["uf2"] = (yield dut.ring_underflow.status)

    run_simulation(dut, tb())

    checks = [
        ("a healthy ring leaves ring_underflow at 0",      out["clean_uf"] == 0),
        ("a healthy ring leaves ring_level_min at 0",      out["clean_min"] == 0),
        ("block_level still CLAMPS the negative level",    out["blk"] == 0),
        ("ring_level_raw reports it unclamped (-64)",      out["raw"] == -64),
        ("ring_underflow counted the episode",             out["uf"] == 1),
        ("ring_level_min remembers the worst (-64)",       out["min"] == -64),
        ("the healthy ring primed first",                  out["primed"] > 0),
        ("ring_unprime saw primed fall",                   out["unprime"] >= 1),
        ("a second episode counts as 2, not as cycles",    out["uf2"] == 2),
    ]
    print("  RING HEALTH CSRs")
    bad = 0
    for name, ok in checks:
        print(f"    {'ok  ' if ok else 'FAIL'}  {name}")
        bad += 0 if ok else 1
    print(f"    raw {out['raw']}, block_level {out['blk']} (the clamp), "
          f"underflow {out['uf']}->{out['uf2']}, min {out['min']}, "
          f"unprime {out['unprime']}")
    return bad


def main():
    dut = build()
    PTR_MOD = 1 << len(dut.dbg_rd)
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
            # 2^32 USED to be the cliff. With PTR_BITS=48 it is an ordinary
            # count, and this row proves it: wr sits exactly on it and the level
            # must come out right without anything special happening.
            cases.append((true_level, OLD_PTR_MOD % PTR_MOD))
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
          f"sides of the 2^{len(dut.dbg_rd)} boundary, and 2^32 is now just a number")
    print()
    fails += health(build())
    print(f"\n{'ALL CHECKS PASSED' if fails == 0 else str(fails) + ' CHECK(S) FAILED'}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
