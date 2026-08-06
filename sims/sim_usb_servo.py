# Closed-loop model of the USB async-feedback servo against the ring.
#
# WHY: with rd advancing every media strobe instead of once per 8-sample block,
# the bench ring oscillates 8..67 against a centre of 64 with ~1400 underrun/s
# and the servo sits at its neutral value. sims/sim_fpp_due.py CANNOT test this
# -- its writer delivers a fixed 48 samples per strobe open-loop, so the ring
# always primes and holds. (An earlier claim that it "reproduced the starved
# FIFO" was wrong: its low readings are the startup ramp, and the trace ends at
# centre.) Nothing in the repo modelled the loop, so the loop is what this adds.
#
# THE SERVO, from usb_avb_subsystem.py:420-460 (Amaranth, outside this repo,
# pre-generated into rtl/usb_avb_subsystem.v):
#
#   fb = (nco_rate << 8) + (err << 6) + (integ >> 6)      err = 64 - block_level
#   integ accumulates ONLY while -32 <= err <= 32          (anti-windup band)
#   integ clamped to +/-(0x2000 << 6)                      (+/-2.08% authority)
#   fb refreshed every 8 SOFs; nco_rate re-measured every 256 SOFs
#
# fb is samples-per-(micro)frame in Q16.16. nco_rate is the MEASURED media
# strobe count, so the base term is self-correcting and P+I only have to centre
# the ring. Nominal 6.0 samples/SOF = 6<<16 = 393216, and 0x2000<<6 >> 6 = 8192
# against that base is 2.08% -- which reproduces the comment in the source and
# is the check that this model has the scaling right.
#
# THE HYPOTHESIS UNDER TEST. block_level = level_samples / 2, so the prime
# hysteresis spans block 8..64 = err 0..56. Over most of that swing |err| > 32
# and the integrator is FROZEN -- exactly when it is needed. Raising the
# un-prime threshold to half-centre (block 32) keeps the excursion inside
# err 0..32 so the integrator can work.
#
# WHAT THIS DOES NOT MODEL: USB transfer jitter, the host's own clock drift, SOF
# jitter, and the packet builder. It models the CONTROL LOOP only. A PASS means
# "the loop is stable and centres", not "the audio is clean".
#
#   run:  python3 sims/sim_usb_servo.py

import sys

# HIGH-SPEED MICROFRAMES, 8 kHz -- not 1 kHz frames. The servo source says its
# integral authority is 2.08%, and (0x2000<<6)>>6 = 8192 is 2.08% of the base
# ONLY if the base is 6<<16 (6 samples per microframe). At 1 kHz the base would
# be 48<<16 and the authority 0.26%, which contradicts the source. This factor
# of 8 is the same one that made an earlier ring sim predict a shift of "28
# addresses" where the bench measured 28 SAMPLES.
SOF_HZ        = 8000.0          # microframes/s (USB high speed)
FS            = 48000.0
NOMINAL_SOF   = FS / SOF_HZ     # 48 samples per frame
Q             = 1 << 16
KI_SHIFT      = 6
INTEG_MAX     = (0x2000 << KI_SHIFT)

CENTRE_BLOCK  = 64              # servo target
PRIME_BLOCK   = 64              # primed sets at/above this block_level
FULL_BLOCK    = 128


def block_of(level_samples):
    """block_level = level_addr >> 4 and level_addr = level_samples * 8."""
    b = int(level_samples // 2)
    return max(0, min(FULL_BLOCK, b))


def run(unprime_block, host_bias=0.0, seconds=20.0, start_level=128.0):
    """Closed loop. host_bias is a systematic delivery error (fraction).

    Returns the per-SOF level history in SAMPLES.
    """
    level = start_level
    primed = True
    integ = 0
    nco_rate = int(NOMINAL_SOF)          # measured strobes per frame
    fb = int(nco_rate * Q)
    hist = []
    strobes = 0.0

    n = int(seconds * SOF_HZ)
    for k in range(n):
        # --- host delivers what the feedback asked for, plus its own error ---
        delivered = (fb / Q) * (1.0 + host_bias)
        level += delivered

        # --- media clock consumes, but ONLY while primed -------------------
        if primed:
            level -= NOMINAL_SOF
        strobes += NOMINAL_SOF

        # --- prime hysteresis (dante_packetizer.py) ------------------------
        b = block_of(level)
        if b >= PRIME_BLOCK:
            primed = True
        elif b < unprime_block:
            primed = False

        # --- servo, every 8 SOFs ------------------------------------------
        if k % 8 == 0:
            err = CENTRE_BLOCK - b
            fb = (nco_rate << 8) + (err << 6) + (integ >> KI_SHIFT)
            if -32 <= err <= 32:                      # ANTI-WINDUP BAND
                integ = max(-INTEG_MAX, min(INTEG_MAX, integ + err))
        # --- nco_rate re-measured every 256 SOFs --------------------------
        if k % 256 == 0:
            nco_rate = int(NOMINAL_SOF)               # measured, self-correcting
        hist.append(level)
    return hist


def report(name, hist, tail_frac=0.5):
    tail = hist[int(len(hist) * tail_frac):]
    lo, hi = min(tail), max(tail)
    avg = sum(tail) / len(tail)
    under = sum(1 for v in tail if block_of(v) < 8)
    return lo, hi, avg, under, (
        f"  {name:<28} level {lo:6.1f}..{hi:6.1f} avg {avg:6.1f} samples"
        f"  (block {block_of(lo):3d}..{block_of(hi):3d} avg {block_of(avg):3d})")


if __name__ == "__main__":
    fails = 0
    print("scaling check against the servo source:")
    print(f"  nominal fb  = {int(NOMINAL_SOF)}<<16 = {int(NOMINAL_SOF)*Q}")
    auth = (INTEG_MAX >> KI_SHIFT) / (int(NOMINAL_SOF) * Q) * 100
    ok = 1.9 < auth < 2.3
    print(f"  {'PASS' if ok else 'FAIL'}  integral authority {auth:.2f}% "
          f"(source comment says 2.08%)")
    if not ok:
        fails += 1

    print("\nHOST DELIVERS EXACTLY WHAT IS ASKED (host_bias = 0):")
    for name, ub in (("un-prime block 8 (old)", 8), ("un-prime block 32 (new)", 32)):
        lo, hi, avg, under, line = report(name, run(ub))
        print(line)

    # The bench shows the host under-delivering ~1.8%. That is the condition the
    # integrator exists to absorb, and the condition under which the anti-windup
    # band matters.
    print("\nHOST UNDER-DELIVERS 1.8% (the measured bench condition):")
    res = {}
    for name, ub in (("un-prime block 8 (old)", 8), ("un-prime block 32 (new)", 32)):
        h = run(ub, host_bias=-0.018)
        lo, hi, avg, under, line = report(name, h)
        res[ub] = (lo, hi, avg, under)
        print(line + f"   underrun frames {under}")

    print()
    old_lo, old_hi, old_avg, old_un = res[8]
    new_lo, new_hi, new_avg, new_un = res[32]
    swing_old, swing_new = old_hi - old_lo, new_hi - new_lo
    c1 = swing_new < swing_old
    print(f"  {'PASS' if c1 else 'FAIL'}  narrower swing: {swing_new:.1f} < "
          f"{swing_old:.1f} samples")
    if not c1:
        fails += 1
    c2 = new_un <= old_un
    print(f"  {'PASS' if c2 else 'FAIL'}  no more underrun frames: "
          f"{new_un} <= {old_un}")
    if not c2:
        fails += 1
    c3 = abs(block_of(new_avg) - CENTRE_BLOCK) < abs(block_of(old_avg) - CENTRE_BLOCK)
    print(f"  {'PASS' if c3 else 'FAIL'}  average nearer centre: block "
          f"{block_of(new_avg)} vs {block_of(old_avg)} (target {CENTRE_BLOCK})")
    if not c3:
        fails += 1

    print("\n  Loop only -- no USB jitter, no host drift, no builder. A PASS means")
    print("  the control loop centres and is stable, NOT that audio is clean.")
    print(f"\n{'ALL CHECKS PASSED' if not fails else str(fails) + ' CHECK(S) FAILED'}")
    sys.exit(1 if fails else 0)
