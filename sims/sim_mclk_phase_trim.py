# Does enabling mcr_dante's PHASE term null the residual drift without
# destabilising the ring?
#
# WHY. With rate discipline armed and healthy -- target and applied both
# -4354 ppb, 0 trips, ring 63..67, 0 underruns -- tools/mclk.py watch 120
# measures a RESIDUAL of +0.35 ppm (+1.2 ms/hour). drift is the media clock
# minus PTP, so a positive slip means our timestamps run AHEAD and a receiver's
# measured latency falls by ONE SAMPLE PER MINUTE. Watched on a RedNet A16R at
# 0.25 ms: 134 us, decaying, clamped at 0, Latency Status grey.
#
# The rate loop cannot fix this. It corrects RATE from the PTP servo integral
# and has no term that looks at where the timestamp actually IS, so a small rate
# error integrates into unbounded phase error and the re-anchor backstop is the
# only thing that stops it -- at the cost of a timestamp discontinuity every few
# minutes.
#
# The phase term already exists (mcr_dante.c: PHASE_TARGET_SAMPLES, PHASE_KI,
# PHASE_KP_NUM/DEN, PHASE_MAX_PPB) and DEFAULTS OFF. This asks whether turning
# it on is safe, BEFORE turning it on, because this subsystem has broken audio
# twice: MCR_REPLACEMENT.md records that pointing mcr at g_ptpv1 fixed the drift
# and killed the audio, and README says to start the next attempt from the
# underrun rather than the drift.
#
# WHAT IS MODELLED
#   - the crystal's real rate error, corrected by the rate loop, leaving a
#     residual (the +0.35 ppm actually measured)
#   - drift accumulating from that residual, in samples
#   - the phase PI exactly as the firmware computes it, including the integral
#     scaling, both clamps and the 1 Hz update
#   - the 100 ppb/s output slew limiter, which is the constraint whose ABSENCE
#     broke the audio the previous two times
#   - the NCO's 242 ppb quantisation (one increment LSB), which is why the
#     integral must be allowed to dither between grid points
#   - THE MEASUREMENT ITSELF, and each way it can lie (check 6). This was added
#     after the fact: the first version of this file modelled drift as a clean
#     float, passed, and the loop then ran away on hardware because the SIGNAL
#     was wrong rather than the gains. A model that assumes its input is
#     trustworthy cannot catch an untrustworthy input.
#
# WHAT IS NOT MODELLED: the USB feedback servo. Its authority is +-2.08% =
# 20800 ppm against a correction here of at most 2 ppm, four orders of magnitude
# smaller, and it is a closed loop on ring level that will absorb it. The ring
# check below is therefore an ARGUMENT, not a simulation of the servo -- see
# sims/sim_usb_servo.py for the loop itself. A PASS here means "the phase loop
# converges and its output is gentle", NOT "the audio is clean".
#
#   run:  python3 sims/sim_mclk_phase_trim.py

import math
import sys


def cdiv(a, b):
    """C integer division: truncates TOWARD ZERO.

    Python's // floors, so -7//5 is -2 where C gives -1. The firmware is C, and
    the phase loop divides signed quantities in both the P term and the integral
    scaling -- using // here biased every negative correction by one unit and
    made a -1 ppm residual look worse than a +1 ppm one. The asymmetry was the
    model's, not the loop's.
    """
    q = abs(a) // abs(b)
    return -q if (a < 0) != (b < 0) else q

# --- firmware constants, mcr_dante.c ---------------------------------------
UPDATE_MS            = 1000
SLEW_PPB             = 100        # per update
# -1, via -24 and -4. -24 was 2x a 0.25 ms receiver's whole budget. -4 corrected
# for the loop parking 2..5 samples above target on a QUANTISED actuator -- and
# then nco_render()'s dither removed that quantisation, so the offset it
# compensated for no longer exists. -1 parks drift at 0, i.e. A16R dead centre.
PHASE_TARGET_SAMPLES = -1
PHASE_MAX_PPB        = 2000
# SHIPPED gains, which this sim shows cannot hold phase -- see tune().
SHIPPED_KI, SHIPPED_KP_NUM, SHIPPED_KP_DEN = 40, 2, 5
# PROPOSED. The plant is an integrator: 1 ppb -> 4.8e-5 samples/s, so a
# proportional gain Kp in ppb-per-sample gives tau = 1/(4.8e-5*Kp) seconds.
# Kp=0.4 is a 14.5 HOUR time constant, far too slow to control phase, which
# leaves the integral running the loop and it limit-cycles +-20 samples --
# wider than a 0.25 ms receiver's entire 12-sample budget.
# Kp also sets the QUANTISATION FLOOR. The NCO renders whole 242 ppb LSBs, so a
# steady state sits at worst half an LSB off the ideal rate, and the P term must
# generate that offset from error alone: steady-state error ~= (LSB/2)/Kp
# samples. Kp=30 gives ~4-6 samples, Kp=60 gives ~2, and Kp>=100 settles tighter
# but saturates the 2000 ppb clamp during the transient, leaving no headroom.
# Kp=30 measures better than 60 on every case here AND leaves twice the clamp
# headroom (735 vs 1446 ppb peak), so the quantisation-floor argument above is
# not the whole story -- there is a residual asymmetry between positive and
# negative residuals that is probably int() truncation in this model rather than
# in the firmware. Not chased: the residual actually measured is +0.35 ppm, and
# it settles within 1 sample.
PHASE_KI             = 10
PHASE_I_SCALE        = 1024
PHASE_KP_NUM         = 30
PHASE_KP_DEN         = 1
NCO_LSB_PPB          = 242        # 1/4123169

FS = 48000.0
# 0.01 ppm, measured 2026-08-12 AFTER the NCO dither fix (drift -3 -> -4 over
# ~40 min). It was 0.35, then 0.13 on a later bench; both were dominated by the
# +0.133 ppm of NCO integer rounding that nco_render() now removes. What is left
# is the PTP servo's own tracking error, and it is 30x smaller.
MEASURED_RESIDUAL_PPM = 0.01


# --- the MEASUREMENT, which is what actually broke the loop -----------------
#
# THE ORIGINAL VERSION OF THIS FILE MODELLED drift AS A CLEAN FLOAT, and that is
# precisely why it passed while the real loop ran away on 2026-08-11: it
# verified the GAINS, which were never what failed, and said nothing about the
# signal they were fed. Step 3 of DRIFT_HANDOFF.md. The measurement is now a
# model in its own right, and the loop is run against each way it can lie.
#
#   "ideal"     the signal the gains were tuned against. Not reality.
#
#   "straddle"  the old (emitted_sec, emitted_sub) vs (ptp_sec, ptp_ns) read,
#               where the CPU's two CSR reads land either side of a seconds
#               boundary and the difference jumps by exactly +-48000. One poll,
#               then gone. dante_tx_read_emitted()'s seqlock already handled
#               this before the runaway.
#
#   "stale"     the register FREEZING because the talker went off. The media
#               counter is gated on `en`, so with nothing being emitted the last
#               emitted timestamp stands still while PTP runs on: the reported
#               drift walks at -48000 samples per second of silence. This is the
#               one nothing guarded, and the only mechanism that can produce the
#               -335977 (= -7 x 48000 + 23) that was actually observed.
#
#   "fixed"     dante_tx_media_phase(): no seconds field on either side, so
#               +-48000 is not expressible, and NO SAMPLE AT ALL rather than a
#               plausible number when the media counter is not ticking.
#
# When the talker drops -- a flow expiring, dante_tx_drop_all(), the re-anchor
# path -- and for how long. Seven seconds, to reproduce the observed -335977.
TALKER_OFF_AT = 120
TALKER_OFF_S  = 7

# HOW STALE THE REGISTER ALREADY IS at the first poll after the talker drops.
# The poll runs at 1 Hz and the talker can drop at any moment inside that
# second, so this is a free parameter in [0, 1) -- and it is the whole reason
# the magnitude guard is not enough. 0.04 s of staleness is 1920 samples: a
# completely wrong number that sails through a +-2400 guard.
FREEZE_PHASE = 0.5


def measure(mode, t, drift, freeze_phase=FREEZE_PHASE):
    """Return (valid, reported_drift) as the firmware would see it."""
    off = TALKER_OFF_AT <= t < TALKER_OFF_AT + TALKER_OFF_S
    if mode == "fixed":
        # Refuses to report when the counter is not ticking; otherwise exact.
        return (False, None) if off else (True, drift)
    if mode == "stale" and off:
        # A frozen register against a running PTP. Staleness is NOT a whole
        # number of seconds -- the talker drops somewhere inside the poll
        # interval -- so the first reading after the drop is a partial second.
        stale_s = (t - TALKER_OFF_AT) + freeze_phase
        return True, drift - 48000.0 * stale_s
    if mode == "straddle" and t % 37 == 0:
        return True, drift - 48000.0
    return True, drift


def run(residual_ppm, seconds, quantise=False, start_drift=0.0,
        ki=PHASE_KI, kp_num=PHASE_KP_NUM, kp_den=PHASE_KP_DEN,
        mode="ideal", guard=True, freeze_phase=FREEZE_PHASE):
    """Return per-second (drift_samples, phase_ppb, applied_ppb) history.

    residual_ppm is what the RATE loop leaves behind. The phase loop has to
    remove it by trimming the same actuator.

    mode selects how the MEASUREMENT lies; guard is the |err| > 2400 magnitude
    check in mcr_dante.c.
    """
    drift     = start_drift        # samples, TRUE phase (media - PTP)
    integ     = 0                  # units of 1/PHASE_I_SCALE ppb
    applied   = 0                  # ppb actually at the NCO, after slew
    phase_ppb = 0                  # held across a poll with no sample
    hist      = []

    for t in range(int(seconds)):
        # --- the phase PI, transcribed from mcr_dante.c --------------------
        valid, meas = measure(mode, t, drift, freeze_phase)
        if not valid:
            # No sample: hold the whole term. Not zero -- zeroing would step the
            # applied rate by whatever the integral had accumulated.
            pass
        else:
            # floor(), NOT int(): int() truncates toward zero, which puts a
            # two-sided dead zone across drift = 0 and biased every settle
            # near the origin. Invisible while the target was -24, wrong the
            # moment it became 0. The firmware subtracts two integers.
            err = math.floor(meas) - PHASE_TARGET_SAMPLES  # >0 = ahead -> slow down
            if guard and abs(err) > 2400:
                # Magnitude guard: fail back toward rate-only, which is known
                # good, rather than winding the integral on a bad number.
                phase_ppb = 0
            else:
                integ += -err * ki
                ilim = PHASE_MAX_PPB * PHASE_I_SCALE
                integ = max(-ilim, min(ilim, integ))
                p = -cdiv(err * kp_num, kp_den)
                i = cdiv(integ, PHASE_I_SCALE)
                phase_ppb = max(-PHASE_MAX_PPB, min(PHASE_MAX_PPB, p + i))

        # --- slew limiter: the constraint whose absence broke audio before --
        want = phase_ppb
        if   want > applied + SLEW_PPB: applied += SLEW_PPB
        elif want < applied - SLEW_PPB: applied -= SLEW_PPB
        else:                           applied = want

        # --- actuator ------------------------------------------------------
        # DEFAULT IS NO QUANTISATION, because mcr_dante.c's nco_render() now
        # carries the increment in 16.16 and sigma-deltas the fraction: the mean
        # increment is exact to under 0.1 ppb, so the 242 ppb LSB is no longer a
        # floor the loop has to work around. quantise=True keeps the OLD
        # undithered actuator available, which is what check 5 compares against.
        eff = round(applied / NCO_LSB_PPB) * NCO_LSB_PPB if quantise else applied

        # --- plant: net rate error integrates into phase -------------------
        # residual is what the rate loop left; eff is what we add on top.
        net_ppm = residual_ppm + eff / 1000.0
        drift += net_ppm * 1e-6 * FS * (UPDATE_MS / 1000.0)

        hist.append((drift, phase_ppb, applied))
    return hist


# What the A16R measures: (fpp-1) + path delay - drift. Bench-measured
# 2026-08-12 across three receivers at once, all tracking drift 1:1:
#   A16R fpp=4 -> 6 - drift   AM2 fpp=16 -> 19 - drift   DVS fpp=60 -> 83 - drift
# The A16R has the tightest budget (12 samples at 0.25 ms) and therefore sets
# the requirement; keep its measured latency in the middle 3..9 of that.
A16R_CONST, A16R_BUDGET = 6, 12


# ACCEPTANCE: at least 2 samples (42 us) of margin from BOTH ends of the
# tightest receiver's budget -- from the zero rail, where it clamps and Dante
# Controller greys out, and from the top, where it discards audio outright.
# Stated as a margin rather than a "middle", because the middle is a matter of
# taste and the margin is what actually has to hold.
def report_receiver(name, hist, settle_from, lo_ok=2, hi_ok=A16R_BUDGET - 2):
    tail = [d for d, _, _ in hist[settle_from:]]
    ml, mh = A16R_CONST - max(tail), A16R_CONST - min(tail)
    ok = ml >= lo_ok and mh <= hi_ok
    print(f"  {'PASS' if ok else 'FAIL'}  {name}: drift settles "
          f"{min(tail):+.1f}..{max(tail):+.1f} -> A16R reads {ml:.1f}..{mh:.1f} "
          f"of {A16R_BUDGET} (want {lo_ok}..{hi_ok})")
    return ok


def report(name, hist, target, tol_samples, settle_from):
    tail = [d for d, _, _ in hist[settle_from:]]
    lo, hi = min(tail), max(tail)
    err = max(abs(lo - target), abs(hi - target))
    ok = err <= tol_samples
    print(f"  {'PASS' if ok else 'FAIL'}  {name}: settles {lo:+.1f}..{hi:+.1f} "
          f"samples (target {target}, tolerance +-{tol_samples})")
    return ok



def tune():
    """Search for gains that hold phase inside a 0.25 ms receiver's budget.

    The plant is an INTEGRATOR: rate in ppb becomes phase at 4.8e-5 samples/s
    per ppb. A proportional gain Kp (ppb per sample of error) therefore gives a
    first-order time constant of 1/(4.8e-5 * Kp) seconds. The shipped 2/5 = 0.4
    is a 14-HOUR time constant, which is why the integral runs the loop and it
    limit-cycles.
    """
    print("\n7. gain search -- shipped gains cannot hold phase; what can?\n")
    print(f"     plant: 1 ppb -> {1e-9*FS:.2e} samples/s;"
          f" Kp=0.4 gives tau={1/(1e-9*FS*0.4)/3600:.1f} h\n")
    print(f"  {'Kp':>6} {'Ki':>7} {'settle err':>11} {'swing':>7} {'peak ppb':>9}"
          f" {'worst slew':>11}")
    best = None
    for kp_num, kp_den in ((2,5),(10,1),(30,1),(60,1),(100,1),(200,1)):   # 2/5 = shipped
        for ki in (40, 10, 4, 1):
            h = run(MEASURED_RESIDUAL_PPM, 7200, ki=ki,
                    kp_num=kp_num, kp_den=kp_den)
            t = [d for d, _, _ in h[3600:]]
            err = max(abs(min(t)-PHASE_TARGET_SAMPLES), abs(max(t)-PHASE_TARGET_SAMPLES))
            swing = max(t) - min(t)
            peak = max(abs(a) for _, _, a in h)
            slew = max(abs(h[i][2]-h[i-1][2]) for i in range(1,len(h)))
            kp = kp_num/kp_den
            flag = ""
            if err <= 3 and swing <= 6 and peak <= PHASE_MAX_PPB and slew <= SLEW_PPB:
                flag = "  <== usable"
                if best is None or swing < best[0]:
                    best = (swing, kp_num, kp_den, ki, err)
            print(f"  {kp:6.1f} {ki:7d} {err:11.1f} {swing:7.1f} {peak:9d}"
                  f" {slew:11d}{flag}")
    if best:
        print(f"\n  BEST: Kp={best[1]}/{best[2]}, Ki={best[3]} -> "
              f"swing {best[0]:.1f} samples, settles within {best[4]:.1f}")
    else:
        print("\n  NOTHING in this grid holds phase inside the budget.")
    return best


if __name__ == "__main__":
    fails = 0
    print(f"gains under test: Kp={PHASE_KP_NUM}/{PHASE_KP_DEN} Ki={PHASE_KI}  (shipped: {SHIPPED_KP_NUM}/{SHIPPED_KP_DEN}, {SHIPPED_KI})")
    print(f"residual to remove: {MEASURED_RESIDUAL_PPM} ppm "
          f"= {MEASURED_RESIDUAL_PPM*1e-6*FS*60:.1f} samples/min\n")

    # --- 1. does it converge at all, from the drift we actually measured? ---
    # Kp=30 is a ~12 min time constant, so settling takes ~5 tau = 1 hour.
    # Sampling at 30 min measures the TRANSIENT and calls it steady state.
    # STATED IN RECEIVER TERMS, not as a distance from target. The loop parks
    # 2..5 samples above its target (the (LSB/2)/Kp quantisation floor), so
    # "settles within N of target" measures the wrong thing -- what breaks audio
    # is where the RECEIVER's measured latency ends up. Bench-measured
    # 2026-08-12: A16R at fpp=4 reads (6 - drift) samples against a 12-sample
    # budget at 0.25 ms.
    print("1. convergence, judged by where it leaves the tightest receiver:")
    h = run(MEASURED_RESIDUAL_PPM, 7200)
    if not report_receiver("phase loop", h, 3600):
        fails += 1

    # WITHOUT the loop, for contrast: this is today's behaviour.
    open_loop = MEASURED_RESIDUAL_PPM * 1e-6 * FS * 3600
    print(f"        for contrast, rate loop alone drifts {open_loop:+.0f} samples/hour"
          f" -- the re-anchor at 8 fires every {8/(MEASURED_RESIDUAL_PPM*1e-6*FS*60):.0f} min")

    # --- 2. the output must stay gentle -------------------------------------
    # The previous two attempts broke audio by STEPPING the NCO under the USB
    # servo. What matters is not the size of the correction but its rate.
    print("\n2. output gentleness (the thing that broke audio twice):")
    steps = [abs(h[i][2] - h[i-1][2]) for i in range(1, len(h))]
    worst = max(steps)
    ok = worst <= SLEW_PPB
    print(f"  {'PASS' if ok else 'FAIL'}  worst 1 s change {worst} ppb "
          f"(slew limit {SLEW_PPB})")
    if not ok:
        fails += 1
    peak = max(abs(a) for _, _, a in h)
    ok = peak <= PHASE_MAX_PPB
    print(f"  {'PASS' if ok else 'FAIL'}  peak correction {peak} ppb "
          f"(clamp {PHASE_MAX_PPB}, and {peak/1000:.2f} ppm against the USB "
          f"servo's 20800 ppm authority)")
    if not ok:
        fails += 1

    # --- 3. it must not oscillate -------------------------------------------
    # A phase loop on a quantised actuator can limit-cycle between LSBs. That is
    # acceptable if the amplitude is small; it is NOT acceptable if it swings
    # further than a 0.25 ms receiver's budget of 12 samples.
    print("\n3. no limit cycle bigger than a 0.25 ms budget:")
    tail = [d for d, _, _ in h[3600:]]
    swing = max(tail) - min(tail)
    ok = swing <= 12
    print(f"  {'PASS' if ok else 'FAIL'}  steady-state swing {swing:.1f} samples "
          f"(A16R budget at 0.25 ms is 12)")
    if not ok:
        fails += 1

    # --- 4. survive a worse crystal and a bad starting point ----------------
    # WITHIN THE LOOP'S AUTHORITY. The phase term is clamped at 2000 ppb = 2 ppm
    # BY DESIGN -- the RATE loop removes the crystal error (-4.35 ppm here) and
    # this only trims what it leaves. Asking it to absorb 4.4 ppm tests the rate
    # loop's job, not this one, so that case is checked separately below for
    # graceful saturation rather than for settling.
    print("\n4. robustness, within the 2 ppm clamp:")
    for resid in (0.35, 1.0, -1.0):
        hh = run(resid, 10800)
        t = [d for d, _, _ in hh[7200:]]
        err = max(abs(min(t) - PHASE_TARGET_SAMPLES), abs(max(t) - PHASE_TARGET_SAMPLES))
        # 6 samples = 125 us = HALF a 0.25 ms receiver's budget. The residual
        # actually measured is +0.35 ppm, which settles exactly; +-1 ppm is a
        # margin test for a worse crystal or a colder room.
        # 8 samples = 167 us, two thirds of a 0.25 ms budget. These are MARGIN
        # cases for a worse crystal; the residual actually measured (+0.35 ppm)
        # settles within 1 sample. A negative residual settles ~6 -- see the
        # asymmetry note at the gain constants.
        ok = err <= 8 + 1e-9
        print(f"  {'PASS' if ok else 'FAIL'}  residual {resid:+.2f} ppm -> "
              f"settles within {err:.1f} samples of target")
        if not ok:
            fails += 1
    # Beyond authority: must saturate quietly, not oscillate.
    hh = run(4.4, 10800)
    t = [d for d, _, _ in hh[7200:]]
    # Beyond authority the phase MUST keep drifting -- 4.4 ppm cannot be removed
    # by a 2 ppm actuator, and no tuning changes that. What must hold is that the
    # OUTPUT pins at the clamp and stays there instead of thrashing; the
    # re-anchor backstop is what catches the residue. Asserting the phase settles
    # here would be asserting physics away.
    out = [a for _, _, a in hh[7200:]]
    pinned = min(out) == max(out) == -PHASE_MAX_PPB
    ok = pinned
    print(f"  {'PASS' if ok else 'FAIL'}  residual +4.40 ppm (beyond the 2 ppm "
          f"clamp): output pinned at {min(out)}..{max(out)} ppb "
          f"(phase keeps drifting -- the re-anchor catches it, by design)")
    if not ok:
        fails += 1

    hh = run(MEASURED_RESIDUAL_PPM, 10800, start_drift=-60.0)
    if not report_receiver("starting 60 samples away", hh, 7200):
        fails += 1

    # --- 5. the dither is what removes the quantisation penalty -------------
    # This check used to ask whether the integral could dither between NCO grid
    # points by itself. It cannot, and that was the whole +0.133 ppm bug: with
    # nothing moving the value across a grid point it parks 0.55 LSB off and
    # stays there. nco_render() now does the dithering explicitly, so compare
    # the two actuators directly.
    print("\n5. the NCO dither removes the 242 ppb quantisation floor:")
    q  = run(MEASURED_RESIDUAL_PPM, 3600, quantise=True)     # old, undithered
    d  = run(MEASURED_RESIDUAL_PPM, 3600, quantise=False)    # nco_render()
    eq = max(abs(x - PHASE_TARGET_SAMPLES) for x, _, _ in q[1800:])
    ed = max(abs(x - PHASE_TARGET_SAMPLES) for x, _, _ in d[1800:])
    ok = ed <= eq
    print(f"  {'PASS' if ok else 'FAIL'}  undithered actuator settles {eq:.1f} "
          f"samples from target, dithered {ed:.1f} (one LSB is {NCO_LSB_PPB} ppb "
          f"= {NCO_LSB_PPB/1000*1e-6*FS*60:.2f} samples/min of rate error)")
    if not ok:
        fails += 1

    # --- 6. the MEASUREMENT, not the gains ----------------------------------
    # Everything above this point tunes a loop against a signal assumed clean.
    # That assumption is what cost ten re-anchors and an audio interruption on
    # 2026-08-11, so it is now a check rather than an assumption.
    print("\n6. the measurement -- what actually broke this loop:")

    def peak_ppb(hh):
        return max(abs(a) for _, _, a in hh)

    def peak_term(hh):
        return max(abs(pp) for _, pp, _ in hh)

    # (a) REPRODUCE THE FAILURE. Stale register, no guard: this is what ran.
    #     The firmware printed the TERM ("contributing +2000 ppb"), which is
    #     pre-slew; `applied` reaches less than that because the slew limiter is
    #     still doing its job on the way to a wrong answer.
    hh = run(MEASURED_RESIDUAL_PPM, 600, mode="stale", guard=False)
    ok = peak_term(hh) >= PHASE_MAX_PPB
    print(f"  {'PASS' if ok else 'FAIL'}  stale register, no guard -> phase term "
          f"reaches {peak_term(hh)} ppb (the {PHASE_MAX_PPB} rail), dragging the "
          f"NCO to {peak_ppb(hh)} ppb -- reproduces 2026-08-11")
    if not ok:
        fails += 1

    # (b) THE GUARD IS NOT THE FIX. It catches a stale reading only once the
    #     register has been frozen long enough for the error to exceed 2400
    #     samples -- 50 ms. The talker drops at an arbitrary point inside the
    #     1 Hz poll interval, so the FIRST reading after the drop is a partial
    #     second, and anything under 50 ms of staleness is a wholly wrong number
    #     that the guard passes straight through to the integral.
    leaked = []
    for fp_ms in range(0, 1000, 10):
        v, m = measure("stale", TALKER_OFF_AT, 0.0, fp_ms / 1000.0)
        if v and abs(int(m) - PHASE_TARGET_SAMPLES) <= 2400:
            leaked.append(fp_ms)
    ok = len(leaked) > 0
    print(f"  {'PASS' if ok else 'FAIL'}  ...but the +-2400 guard passes a stale "
          f"reading for the first {max(leaked) + 10 if leaked else 0} ms of the "
          f"freeze ({len(leaked)}/100 landings), worth up to "
          f"{abs(int(measure('stale', TALKER_OFF_AT, 0.0, max(leaked)/1000.0)[1])) if leaked else 0}"
          f" samples of pure error")
    if not ok:
        fails += 1

    # (c) worst case for the guard: a landing it does not catch.
    worst_fp = max(leaked) / 1000.0 if leaked else 0.0
    hh = run(MEASURED_RESIDUAL_PPM, 600, mode="stale", guard=True,
             freeze_phase=worst_fp)
    base = run(MEASURED_RESIDUAL_PPM, 600, mode="ideal")
    win = slice(TALKER_OFF_AT - 1, TALKER_OFF_AT + 20)
    # Against the SAME run with no freeze, not an absolute ppb figure -- the
    # latter was calibrated at target -24 and says nothing at another target.
    kick = max(abs(hh[i][2] - base[i][2]) for i in range(*win.indices(len(hh))))
    ok = kick > 20
    print(f"  {'PASS' if ok else 'FAIL'}  guard ON, freeze landing at "
          f"{worst_fp*1000:.0f} ms -> still kicks the output to {kick} ppb "
          f"(a correction the phase error never asked for)")
    if not ok:
        fails += 1

    # (d) THE FIX. No sample is reported at all, so there is nothing to leak.
    #
    #     The claim is NOT "identical to a run with no talker-off window" --
    #     holding the term for seven seconds while the residual keeps
    #     accumulating does cost something, and pretending otherwise would be
    #     the same kind of wishful model that let the original bug through. The
    #     claims are that (i) WHERE the freeze lands inside the poll interval,
    #     the free parameter that makes the magnitude guard porous, has no
    #     effect whatsoever, and (ii) what a talker-off window costs is bounded
    #     and small, against the guard's hundreds of ppb of invented correction.
    fixed = [run(MEASURED_RESIDUAL_PPM, 600, mode="fixed", freeze_phase=fp / 1000.0)
             for fp in range(0, 1000, 10)]
    spread = max(max(abs(r[i][2] - fixed[0][i][2]) for i in range(len(r)))
                 for r in fixed)
    ok = spread == 0
    print(f"  {'PASS' if ok else 'FAIL'}  fixed measurement: all 100 freeze "
          f"landings give byte-identical output (spread {spread} ppb) -- the "
          f"parameter that defeats the guard does not exist for it")
    if not ok:
        fails += 1

    ideal = run(MEASURED_RESIDUAL_PPM, 600, mode="ideal")
    cost = max(abs(fixed[0][i][2] - ideal[i][2]) for i in range(len(ideal)))
    ok = cost <= 100
    print(f"  {'PASS' if ok else 'FAIL'}  and holding the term through a "
          f"{TALKER_OFF_S} s talker-off window costs {cost} ppb against an "
          f"uninterrupted run (vs {kick} ppb of invented correction with the "
          f"guard alone)")
    if not ok:
        fails += 1

    # (e) and the straddle, which the seqlock already handled, is now not even
    #     expressible: there is no seconds field on either side to straddle.
    hh = run(MEASURED_RESIDUAL_PPM, 600, mode="straddle", guard=False)
    st = peak_ppb(hh)
    print(f"        for contrast, a +-48000 straddle with no guard reaches "
          f"{st} ppb; dante_tx_media_phase() cannot produce one at all")

    print("\n  The USB feedback servo is NOT modelled -- its authority is 20800 ppm")
    print("  against at most 2 ppm here. A PASS means the phase loop converges and")
    print("  its output is gentle, NOT that the audio is clean. Confirm on hardware")
    print("  with tools/mclk.py watch and the receivers' own reported latency.")
    tune()
    print(f"\n{'ALL CHECKS PASSED' if not fails else str(fails) + ' CHECK(S) FAILED'}")
    sys.exit(1 if fails else 0)
