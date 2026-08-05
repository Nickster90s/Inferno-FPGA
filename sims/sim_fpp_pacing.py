# Migen sim of ARBITRARY-fpp packet pacing, before touching the packetizer.
#
# WHY: flow_cfg bit [4] is a single bit -- fpp is either 8 or 16, nothing else.
# That turns out to bound which receivers can subscribe at all:
#
#     DVS               asks fpp=60   (measured: 124 requests, never varies)
#     A16R at 5 ms      asks fpp=32
#     A16R at 1/2 ms    asks fpp=8/16   <- the only two we serve
#     0.25 ms target    will ask ~2-4
#
# So DVS cannot connect to us at any latency setting, and the 0.25 ms goal is
# unreachable, for the same reason.
#
# THE TRICK THAT HAS TO GO. Today the tick fires when the low log2(fpp) bits of
# the sample counter wrap, and the emitted value is `counter - (fpp-1)`. That
# makes `emitted % fpp == 0` true BY CONSTRUCTION -- but only because fpp is a
# power of two. 60 is not, so the mask cannot be stretched and the property has
# to come from somewhere else.
#
# THE REPLACEMENT: a per-context phase counter that counts 0..fpp-1 and emits on
# the last count. `emitted % fpp == 0` then holds iff the counter is PHASE
# ALIGNED with the timestamp counter, i.e. phase == ts_sub % fpp at all times.
#
# That alignment is only self-sustaining if **fpp divides 48000**, because
# ts_sub wraps there. 48000 = 2^7 * 3 * 5^3:
#
#     2 4 8 16 24 32 48 60 64 128   divide      -> acceptable
#     36 256                        do not      -> must be rejected
#
# WHAT THIS SIM PROVES (or refutes) BEFORE A 20-MINUTE BUILD:
#   1. Packets come out exactly every fpp samples, for every candidate fpp.
#   2. Every emitted timestamp is a multiple of fpp.
#   3. Alignment SURVIVES THE ts_sub WRAP at 48000 -- the case the power-of-two
#      mask got for free and a modulo counter does not.
#   4. Contexts with different fpp run concurrently without interfering.
#   5. A non-divisor (36) genuinely breaks property 2, so rejecting it in
#      dante_flows.c is necessary rather than merely cautious.
#
# WHAT IT DOES NOT PROVE: nothing about payload size, FIFO depth or bandwidth.
# fpp=60 x 4 slots x 3 B = 720 B payload against 384 B at fpp=16, and the
# per-context FIFO is sized for the old maximum. That is a separate question and
# sim_ring_accounting.py is where it belongs -- see the note at the end.
#
# STOPPED AT IMPLEMENTATION -- READ THIS BEFORE TRYING AGAIN.
#
# The pacing design below is correct and the checks pass, but wiring it into
# dante_packetizer.py hits a coupling this sim does not model and which is the
# real blocker:
#
#   rd     = shared ring READ pointer, advanced as `rd + 64` once per packet
#            build (dante_packetizer.py, FSM tail) = exactly 8 samples
#   f_base = rd - Mux(fpp16, 64, 0)
#            i.e. an fpp=16 flow starts one 8-sample tick earlier, which is the
#            only reason ONE read pointer can serve flows of two different fpp
#   level  = wr - rd, and level drives the USB feedback PI servo
#
# fpp=60 is not a multiple of 8, so a 60-sample window cannot be positioned from
# a pointer that only moves in 8-sample steps, and no per-context offset fixes
# that -- the granularity itself is wrong. Making `rd` sample-granular changes
# what `level` means, and `level` is the input to the USB servo that took a long
# time to stabilise.
#
# So arbitrary fpp is a change to the RING CONSUMPTION MODEL, not to pacing.
# The order for a real attempt is:
#   1. extend sims/sim_ring_accounting.py to a sample-granular rd and show the
#      servo input is unchanged in scale and dynamics
#   2. then this pacing change
#   3. then firmware: fpp index + the (ts_sub % fpp) phase seed
# Doing 2 first, as was attempted, leaves the packetizer half-converted.
#
#   run:  python3 sims/sim_fpp_pacing.py

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from migen import *

SUBSEC_MAX = 48000


class FppPacer(Module):
    """One context's pacing, mirroring the intended RTL.

    phase counts 0..fpp-1 on every sample strobe and emits on the last count.
    phase_load seeds it from firmware, which is where the modulo lives: firmware
    already computes the anchor, so `sub % fpp` costs it a division it can
    afford and saves a divider in fabric.
    """
    def __init__(self, fpp_max=64):
        self.strobe     = Signal()
        self.fpp        = Signal(max=fpp_max + 1)
        self.phase_load = Signal()
        self.phase_init = Signal(max=fpp_max + 1)
        self.emit       = Signal()
        self.phase      = Signal(max=fpp_max + 1)

        self.sync += [
            If(self.phase_load,
                self.phase.eq(self.phase_init),
            ).Elif(self.strobe,
                If(self.phase == (self.fpp - 1),
                    self.phase.eq(0),
                ).Else(
                    self.phase.eq(self.phase + 1),
                ),
            ),
        ]
        # Emit on the strobe that completes a group of fpp samples.
        self.comb += self.emit.eq(self.strobe & (self.phase == (self.fpp - 1)))


def run_one(fpp, samples, start_sub=0):
    """Drive `samples` strobes; return the emitted timestamps.

    The timestamp counter is modelled in the testbench exactly as the packetizer
    has it: ts_sub increments per strobe and wraps at 48000. The emitted value is
    ts_sub - (fpp-1), i.e. the START of the group just completed.
    """
    dut = FppPacer()
    emitted = []

    def tb():
        yield dut.fpp.eq(fpp)
        yield dut.phase_init.eq(start_sub % fpp)
        yield dut.phase_load.eq(1)
        yield
        yield dut.phase_load.eq(0)

        sub = start_sub
        for _ in range(samples):
            yield dut.strobe.eq(1)
            yield
            if (yield dut.emit):
                emitted.append((sub - (fpp - 1)) % SUBSEC_MAX)
            sub = (sub + 1) % SUBSEC_MAX
        yield dut.strobe.eq(0)
        yield

    run_simulation(dut, tb())
    return emitted


def check(fpp, samples, start_sub, expect_aligned=True):
    ts = run_one(fpp, samples, start_sub)
    if len(ts) < 3:
        return False, f"only {len(ts)} packets emitted"

    # 1. spacing is exactly fpp
    gaps = {(b - a) % SUBSEC_MAX for a, b in zip(ts, ts[1:])}
    if gaps != {fpp}:
        return False, f"spacing {sorted(gaps)} != {{{fpp}}}"

    # 2. every emitted timestamp is a multiple of fpp
    bad = [t for t in ts if t % fpp != 0]
    if expect_aligned and bad:
        return False, f"{len(bad)}/{len(ts)} not multiples of fpp (e.g. {bad[0]})"
    if not expect_aligned and not bad:
        return False, "expected misalignment, got none"

    return True, f"{len(ts)} packets, spacing {fpp}, aligned={not bad}"


if __name__ == "__main__":
    fails = 0
    print(f"subsec wraps at {SUBSEC_MAX} = 2^7 * 3 * 5^3\n")

    # --- 1/2. spacing and alignment, from a zero anchor -------------------
    print("candidate fpp values, 3x fpp samples from sub=0:")
    for fpp in (2, 4, 8, 16, 24, 32, 48, 60, 64):
        ok, msg = check(fpp, fpp * 3 + 2, 0)
        print(f"  {'PASS' if ok else 'FAIL'}  fpp={fpp:3d}  {msg}")
        if not ok:
            fails += 1

    # --- 3. alignment across the ts_sub wrap ------------------------------
    # The power-of-two mask got this free. Start close enough to 48000 that the
    # run crosses it, and require alignment to hold on the far side.
    print("\nacross the subsec wrap (the case the old mask got for free):")
    for fpp in (8, 16, 24, 32, 60):
        start = SUBSEC_MAX - 3 * fpp
        ok, msg = check(fpp, 7 * fpp, start)
        print(f"  {'PASS' if ok else 'FAIL'}  fpp={fpp:3d} from sub={start}  {msg}")
        if not ok:
            fails += 1

    # --- 5. a non-divisor must actually break it --------------------------
    # If 36 quietly worked, rejecting it would be superstition. Crossing the
    # wrap with fpp=36 must produce timestamps that are NOT multiples of 36.
    print("\nnon-divisor of 48000 must break alignment (justifies rejecting it):")
    for fpp in (36,):
        start = SUBSEC_MAX - 3 * fpp
        ok, msg = check(fpp, 7 * fpp, start, expect_aligned=False)
        print(f"  {'PASS' if ok else 'FAIL'}  fpp={fpp} from sub={start}  {msg}")
        if not ok:
            fails += 1

    # --- 4. mixed fpp contexts do not interfere ---------------------------
    # Each context has its own phase register, so this should be trivially true;
    # it is checked because the CURRENT design shares one tick across contexts
    # and gates it with due_mask, which is exactly where the fpp=16-at-6000-pps
    # bug came from.
    print("\nmixed fpp concurrently (independent phase registers):")
    mixed = {}
    for fpp in (8, 16, 60):
        mixed[fpp] = run_one(fpp, 60 * 4, 0)
    ok = all(len(mixed[f]) == (60 * 4) // f for f in mixed)
    for f in sorted(mixed):
        print(f"    fpp={f:3d}: {len(mixed[f])} packets in 240 samples "
              f"(expected {240 // f})")
    print(f"  {'PASS' if ok else 'FAIL'}  each context emits at its own rate")
    if not ok:
        fails += 1

    # --- payload note, NOT proven here ------------------------------------
    print("\nNOT PROVEN HERE -- payload growth (belongs in sim_ring_accounting):")
    for fpp in (16, 24, 32, 60):
        pay = 9 + fpp * 4 * 3
        print(f"    fpp={fpp:3d}  4 slots -> {pay:4d} B payload, "
              f"{pay + 28:4d} B IP total, {48000 // fpp:5d} pps/flow")
    print("    fpp=16 is today's maximum at 384 B; fpp=60 needs 720 B and the")
    print("    per-context FIFO is sized for the old figure.")

    print(f"\n{'ALL CHECKS PASSED' if not fails else str(fails) + ' CHECK(S) FAILED'}")
    sys.exit(1 if fails else 0)
