# Does making the ring read pointer SAMPLE-GRANULAR disturb the USB servo?
#
# WHY THIS EXISTS. Arbitrary fpp (DVS asks for 60) is blocked not by pacing --
# sim_fpp_pacing.py proves that design -- but by the ring:
#
#   rd     advances +64 once per BLOCK TRAVERSAL = 8 sample-times x 8 channels
#   f_base = rd - Mux(fpp16, 64, 0)
#   level  = wr - rd, and level is the input to the USB feedback PI servo
#
# 60 is not a multiple of 8, so a 60-sample window cannot be positioned from a
# pointer that only moves in 8-sample steps. The fix is to advance rd by +8 per
# MEDIA STROBE instead of +64 per block, and set f_base = rd - fpp*8.
#
# The risk is not correctness of the average -- both disciplines consume 8
# addresses per sample-time by construction -- it is that `level` changes SHAPE.
# The servo was tuned against the block-quantised version and took a long time
# to stabilise; a different ripple could destabilise it. That is what this
# measures, and it is the question that must be answered BEFORE the packetizer
# is touched, because getting it wrong is audible.
#
# WHAT THIS MEASURES
#   1. Mean level is identical under both disciplines (sanity: same average).
#   2. Peak-to-peak ripple. The block-quantised reader takes 8 sample-times of
#      data in one instant, so level saws by 64 addresses = 8 samples. The
#      sample-granular reader should ripple LESS, not more.
#   3. RING DEPTH. A flow with fpp=F reads the F samples ENDING at rd, so the
#      ring must retain F samples of history plus whatever the level swings.
#      fifo_depth is 64 samples per channel today; fpp=60 leaves 4.
#
# WHAT THIS DOES NOT MEASURE: the servo's closed-loop response. This is an
# open-loop shape comparison. A previous attempt in this project inferred a
# closed-loop conclusion from an open-loop measurement ("the median filter is
# pure lag") and was wrong. Treat a PASS here as "does not obviously disturb the
# servo", not as "the servo is fine".
#
#   run:  python3 sims/sim_ring_sample_rd.py

import sys

# One "sample-time" is one media strobe. Addresses are in ring-slot units:
# 8 channels per ring, so one sample-time of data is 8 addresses.
CH_PER_RING = 8


def simulate(discipline, fpp, n_samples, prime_level=64, jitter=None):
    """Return the per-sample-time history of `level` (= wr - rd), in addresses.

    writer:  +8 every sample-time (the USB side, nominally rate-locked)
    reader:  'block'  -> +64 every 8 sample-times   (today)
             'sample' -> +8 every sample-time       (proposed)

    Both are gated on `primed`, exactly as the packetizer holds rd while the
    ring is below its prime floor.
    """
    wr = prime_level * CH_PER_RING
    rd = 0
    levels = []
    for t in range(n_samples):
        # --- writer -------------------------------------------------------
        step = CH_PER_RING
        if jitter is not None:
            step += jitter[t % len(jitter)]
        wr += step

        # --- reader -------------------------------------------------------
        primed = (wr - rd) >= CH_PER_RING * fpp      # need a full window
        if primed:
            if discipline == "block":
                if (t % 8) == 7:
                    rd += 64
            else:
                rd += CH_PER_RING
        levels.append(wr - rd)
    return levels


def stats(levels):
    lo, hi = min(levels), max(levels)
    mean = sum(levels) / len(levels)
    return lo, hi, mean, hi - lo


if __name__ == "__main__":
    fails = 0
    N = 20000

    print("level is in ADDRESSES; one sample-time = 8 addresses\n")
    print(f"{'discipline':>10} {'fpp':>4} {'min':>7} {'max':>7} {'mean':>9} "
          f"{'p-p':>6} {'p-p samples':>12}")

    results = {}
    for disc in ("block", "sample"):
        for fpp in (8, 16):
            lv = simulate(disc, fpp, N)
            lo, hi, mean, pp = stats(lv)
            results[(disc, fpp)] = (lo, hi, mean, pp)
            print(f"{disc:>10} {fpp:4d} {lo:7d} {hi:7d} {mean:9.1f} "
                  f"{pp:6d} {pp / CH_PER_RING:12.1f}")

    # 1. same mean
    print()
    for fpp in (8, 16):
        mb = results[("block", fpp)][2]
        ms = results[("sample", fpp)][2]
        ok = abs(mb - ms) <= CH_PER_RING * 8      # within one block
        print(f"  {'PASS' if ok else 'FAIL'}  fpp={fpp}: mean level "
              f"block {mb:.1f} vs sample {ms:.1f} (diff {abs(mb - ms):.1f})")
        if not ok:
            fails += 1

    # 2. ripple must not grow
    for fpp in (8, 16):
        pb = results[("block", fpp)][3]
        ps = results[("sample", fpp)][3]
        ok = ps <= pb
        print(f"  {'PASS' if ok else 'FAIL'}  fpp={fpp}: ripple sample-granular "
              f"{ps} <= block {pb}  ({'smoother' if ps < pb else 'same'})")
        if not ok:
            fails += 1

    # 2b. same, with a jittery writer -- the USB side is not perfectly regular
    print("\nwith a jittery writer (+/-1 address per sample-time):")
    jit = [1, 0, -1, 0, 1, 1, -1, -1, 0, 0]
    for fpp in (8, 16, 60):
        row = []
        for disc in ("block", "sample"):
            lv = simulate(disc, fpp, N, jitter=jit)
            row.append(stats(lv))
        (_, _, mb, pb), (_, _, ms, ps) = row
        ok = ps <= pb + CH_PER_RING
        print(f"  {'PASS' if ok else 'FAIL'}  fpp={fpp:3d}: p-p block {pb:5d} "
              f"vs sample {ps:5d}")
        if not ok:
            fails += 1

    # 3. RING DEPTH -- the finding that actually gates fpp=60
    print("\nring depth needed: a flow reads the fpp samples ENDING at rd,")
    print("so the ring must retain fpp samples plus the level swing.")
    FIFO_DEPTH = 64          # samples per channel, dante_packetizer default
    print(f"\n{'fpp':>5} {'needs':>7} {'have':>6} {'margin':>8}   verdict")
    for fpp in (8, 16, 24, 32, 48, 60):
        swing = 6            # samples; measured ring level 59..69 on hardware
        need = fpp + swing
        margin = FIFO_DEPTH - need
        verdict = "OK" if margin >= 8 else ("TIGHT" if margin >= 0 else "TOO SMALL")
        print(f"{fpp:5d} {need:7d} {FIFO_DEPTH:6d} {margin:8d}   {verdict}")
        if margin < 8:
            fails += 1 if margin < 0 else 0

    print()
    print("  CONCLUSION: fifo_depth=64 does NOT leave usable margin at fpp=60")
    print("  (60 + 6 = 66 > 64, i.e. already negative). fifo_depth must rise to")
    print("  128 for fpp=60, which doubles the per-ring BRAM: 8 rings x 128 x 8")
    print("  samples. That is a resource question for the build, not a timing one.")
    print()
    print("  NOTE the writer here is nominally rate-locked and the reader is")
    print("  gated only on having a full window. The real prime floor, the trip")
    print("  guard and the servo are NOT modelled -- see the header.")

    print(f"\n{'ALL CHECKS PASSED' if not fails else str(fails) + ' CHECK(S) FAILED'}")
    sys.exit(1 if fails else 0)
