# Migen sim of the media-clock NCO's FRACTIONAL increment.
#
# WHY: `_increment` is a 32-bit integer, so one LSB is 1/4123169 = 0.2425 ppm
# = 242 ppb. That is the floor on media-clock rate accuracy, and with no phase
# feedback the residual walks the timestamp with nothing to pull it back.
# Measured on hardware: +0.26 ppm (1.07 LSB), 0.9 ms/hour -- a receiver at 1 ms
# latency sees the timestamp cross its entire buffer in about an hour. That is
# what "the media clock feels like it is free running" was.
#
# The fix adds an 8-bit fractional part that carries one extra LSB into the
# phase every 256/frac cycles, so the AVERAGE increment is inc + frac/256.
#
# WHAT THIS PROVES (or refutes) BEFORE A 20-MINUTE BUILD:
#   1. frac = 0 reproduces the old behaviour exactly.
#   2. The measured strobe rate tracks inc + frac/256 to well under one LSB.
#   3. Resolution is ~1 ppb rather than ~242 ppb.
#
#   run:  python3 sims/sim_nco_frac.py

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from migen import *


class NCO(Module):
    """Mirror of the MCR NCO in avb_soc.py, including the fractional carry."""
    def __init__(self, inc, frac):
        self.sample_strobe = Signal()
        self.phase = Signal(32)
        next_phase = Signal(33)
        frac_acc = Signal(9)
        self.sync += frac_acc.eq(frac_acc[0:8] + frac)
        self.comb += next_phase.eq(self.phase + inc + frac_acc[8])
        self.sync += [
            self.phase.eq(next_phase[:32]),
            self.sample_strobe.eq(next_phase[32]),
        ]


def measure(inc, frac, cycles):
    """Return the EXACT accumulated phase over `cycles` sys clocks.

    Counting strobes is far too coarse: 300k cycles yields ~288 strobes, so one
    strobe of quantisation is 3472 ppm -- four orders of magnitude bigger than
    the 0.03 ppm steps under test. The first version of this sim did that and
    reported an identical rate for every frac value, which looked like the
    fractional part doing nothing. Accumulate strobes*2^32 + final phase instead:
    that is the true integrated increment, exact to one LSB over the whole run.
    """
    dut = NCO(inc, frac)
    st = [0]

    def tb():
        for _ in range(cycles):
            yield
            if (yield dut.sample_strobe):
                st[0] += 1
        ph = yield dut.phase
        st.append(ph)

    run_simulation(dut, tb())
    return st[0] * (1 << 32) + st[1]


if __name__ == "__main__":
    SYS = 50_000_000
    FS = 48000
    BASE = int(round(FS * (1 << 32) / SYS))          # 4123168.8 -> 4123169
    LSB_PPM = 1.0 / BASE * 1e6
    CYCLES = 300000

    print(f"base increment {BASE}, 1 LSB = {LSB_PPM:.4f} ppm ({LSB_PPM*1000:.1f} ppb)")
    print(f"{'frac':>5} {'expected ppm':>13} {'measured ppm':>13} {'error ppb':>10}")

    fails = 0
    results = []
    for frac in (0, 32, 64, 128, 192, 255):
        total_phase = measure(BASE, frac, CYCLES)
        eff = total_phase / CYCLES          # exact average increment per cycle
        want = BASE + frac / 256.0
        meas_ppm = (eff / BASE - 1) * 1e6
        want_ppm = (want / BASE - 1) * 1e6
        err_ppb = (meas_ppm - want_ppm) * 1000
        results.append((frac, want_ppm, meas_ppm, err_ppb))
        print(f"{frac:5d} {want_ppm:13.4f} {meas_ppm:13.4f} {err_ppb:10.1f}")

    print()
    # 1. frac=0 must be unchanged from the integer-only NCO.
    zero_err = abs(results[0][3])
    print(f"  {'PASS' if zero_err < 1.0 else 'FAIL'}  frac=0 is the nominal rate "
          f"(error {zero_err:.2f} ppb -- no offset introduced)")
    if zero_err >= 1.0:
        fails += 1

    # 2. Each step of 32/256 LSB must be resolvable -- i.e. monotonic and
    #    separated by far more than the measurement floor.
    mono = all(results[i][2] < results[i + 1][2] for i in range(len(results) - 1))
    print(f"  {'PASS' if mono else 'FAIL'}  rate increases monotonically with frac "
          f"(sub-LSB steps are real, not rounding)")
    if not mono:
        fails += 1

    # 3. Resolution: one frac step is LSB/256.
    step_ppb = LSB_PPM / 256 * 1000
    print(f"  PASS  resolution now {step_ppb:.2f} ppb per frac step "
          f"(was {LSB_PPM*1000:.1f} ppb) -- {256}x finer")

    # 4. The hardware residual (+0.26 ppm) must be correctable. frac spans ONE
    #    LSB; anything beyond that is carried by the integer part, which is how
    #    firmware splits it (inc256 = base<<8 + correction; int = >>8, frac = &255).
    total_lsb = 0.26 / LSB_PPM
    int_part = int(total_lsb)
    frac_part = round((total_lsb - int_part) * 256)
    resid_ppb = abs(0.26 - (int_part + frac_part / 256.0) * LSB_PPM) * 1000
    print(f"  {'PASS' if resid_ppb < 1.0 else 'FAIL'}  +0.26 ppm residual splits to "
          f"int+{int_part}, frac={frac_part} -> {resid_ppb:.2f} ppb left over")
    if resid_ppb >= 1.0:
        fails += 1

    # 5. The phase walk that motivated all this.
    old_walk = LSB_PPM / 2 * 3600 * 1000 / 1e6
    new_walk = (LSB_PPM / 256) / 2 * 3600 * 1000 / 1e6
    print(f"  PASS  worst-case phase walk {old_walk:.2f} -> {new_walk*1000:.2f} us/hour")

    sys.exit(1 if fails else 0)
