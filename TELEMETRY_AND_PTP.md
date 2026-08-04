# Proper FPGA readout, and a PTP servo worth trusting

Written after a session where three wrong conclusions all traced to the same
cause: no way to see inside the running device. Diagnosis was a 200-byte UDP
snapshot plus pasted picocom scrollback.

## What went wrong without it

- Underruns attributed to a code change; they were not growing in the baseline
  either. Reverted a working drift fix for nothing.
- 16506 pps read off hand-indexed stats offsets that had gone stale. The rate
  was a correct 9014.
- A "25x drift improvement" credited to a mechanism that the [inc] experiment
  later proved never executed -- the NCO increment never changed once.
- PTP cycling lock/unlock every ~30 s went unnoticed for hours because nothing
  reported lock transitions; it was found only when a console log happened to be
  pasted.

Each cost a flash-and-measure cycle or worse. The instrument was the bottleneck.

## 1. Streaming telemetry, not snapshots

A snapshot cannot show a servo. Add a UDP telemetry stream: a fixed-size record
pushed on every PTP event, into a ring, drained by the host.

Per Sync/DelayResp record (~32 bytes):

    seq, t1 (master tx), t2 (our rx), t3 (our tx), t4 (master rx),
    raw offset, filtered offset, path delay, addend, freq_integral,
    lock state, flags (outlier rejected / follow-up missing / mispaired)

At 1-2 Sync/s that is trivial bandwidth, and it makes the servo analysable
OFFLINE: plot offset vs time, histogram the outliers, correlate rejections with
mac_writer_err. None of that is possible today.

Same pattern for the media clock: push (ts_sec, ts_sub, ptp_now, nco_increment,
fifo_level, underrun, overrun) at 10 Hz. The underrun-with-fifo-at-centre
contradiction would have been solved in one capture.

Rules learned the hard way:
- NEVER hand-index the reply. Emit a version + field count, and have the host
  parse by name. Two wrong conclusions came from stale offsets.
- Log STATE TRANSITIONS, not just levels. Lock/unlock, talker on/off, anchor,
  flow bind/unbind -- each with a timestamp. Levels hide events.
- Watch the reply size: growing the stats reply 200 -> 208 bytes silently killed
  the port. Cause still unknown; a streaming channel sidesteps it.

## 2. A PTP servo worth trusting

We already have the hard part: hardware timestamping in the TSU. What is missing
is everything around it.

Present state: PI on the filtered offset, median filter, fixed thresholds,
lock by streak. Excursions of +/-5-10 us appear against a sub-microsecond steady
state -- bad measurements, not clock movement. Now suppressed by unlock
hysteresis and outlier rejection, but the SOURCE is unaddressed.

Where the accuracy actually goes, in order:

1. **RX frame loss.** mac_writer_err climbs ~25/s under multicast flood. A lost
   FollowUp or DelayResp mispairs with the wrong Sync and injects microseconds
   of error. THIS IS THE ROOT CAUSE and it is a gateware fix: the rx_gate MAC
   allow-list from the plan's risk 8. Nothing else matters until it is done.

   **DONE and MEASURED (2026-08-01) — see RX_GATE.md.** A/B on one bitstream,
   filter toggled at runtime: **84 of 400 unicast round-trips lost with it off,
   0 of 400 with it on.** The flood (3000.7/s) is fully gated.

   Two corrections to the framing above, both learned by measuring:

   - **mac_writer_err was the wrong instrument.** It counts a frame aborted
     while the 2-deep status FIFO is full, *before* the destination MAC is
     classified, so after rx_gate ~98% of what it counts is flood we were
     discarding anyway. It falls 61.1 -> 8.4/s, not to 0, while actual loss goes
     21% -> 0%. Round-trip loss is the honest number. Making the counter mean
     what its name says is a small deferred-increment change in liteeth's
     sram.py, not done here.
   - **The outlier SOURCE is now gated, but the outlier RATE is not yet
     re-measured.** No unlocks or re-anchors occurred in either 90 s arm, and no
     +/-5-10 us excursions were seen (offset stayed inside a ~200-300 ns band
     both ways) -- but the outliers were always intermittent and are masked by
     hysteresis, so confirming they have stopped needs a long console watch for
     `[ptpv1] outlier ... ignored`, which the operator's picocom sees and this
     host does not.
2. **PTPv1 has no correctionField.** Switch residence time lands directly in the
   offset -- the one place Audinate is not better, they have the same problem on
   PTPv1. On a quiet switch this is tens to hundreds of ns.
3. **Sync rate.** 1-2/s against gPTP's 8/s. Every servo constant inherited from
   the gPTP tuning is wrong at this rate; KI especially.
4. **Asymmetry.** t2-t1 and t4-t3 paths are assumed equal. A fixed asymmetry is a
   constant offset -- harmless for audio, since receivers absorb it as latency.

Realistic target: with rx_gate done and the servo retuned for the PTPv1 rate,
sub-100 ns sustained is reachable with hardware timestamps. "Better than
Audinate" is plausible on offset stability precisely because we control the
whole path; it is NOT plausible on interop, which is where their value is.

## Suggested order

1. rx_gate -- stop losing frames. Fixes the outlier SOURCE, not the symptom.
2. Streaming telemetry -- make the servo visible before touching it.
3. Retune the servo for 1-2 Sync/s with the data from 2.
4. Media clock: it is currently not disciplined AT ALL (see MCR_REPLACEMENT.md).
5. Drift last -- it is a consequence of 4, not an independent problem.

Do NOT tune the servo before 1 and 2. Today's session is the argument: every
change made without an instrument had to be reverted.

---

# BUILT AND MEASURING (2026-08-04) — `telem.c` + `tools/telemetry.py`

Item 2 done. Fixed-size records pushed ON EVENT into a 96-entry ring, drained
over UDP 7779 opcode `'t'`, parsed by name against a `'TLM1'` version tag,
logged to CSV for offline analysis. Separate opcode with its own bounded reply,
so it cannot regress the 200-byte stats endpoint that died once at 208.

Covers **both error axes** and state transitions:

| record | carries |
|---|---|
| `ptp` (per servo update) | raw offset, filtered offset, path delay, rate_ppb, locked/outlier flags |
| `mclk` (1 Hz) | **drift_samples (PHASE)** + applied_ppb (RATE), ring min/avg/max, underrun/s |
| `event` | lock, unlock, anchor, talker on/off, flow bind/unbind, mclk arm/disarm/trip |

Sequence gaps are reported, never silently skipped.

## What it found in the first 105 seconds

**1. The Sync rate is 3.08/s, not ~2 Hz.** `SERVO_KI_NUM` was cut 3600 -> 900
"for ~2 Hz" — a guess that was never measured. At the real 3.08 Hz the integral
gain is under-scaled by about 1.5x.

**2. The median filter is pure lag.** Measured over 217 steady-state samples:

    raw offset      sd = 152 ns
    filtered offset sd = 158 ns      <- filtering makes it WORSE
    outliers flagged: 0

MEDIAN_N=7 at 3.08 Hz costs ~2.3 s of phase lag in the servo loop and removes
no noise. It was added to suppress the +/-5-10 us excursions caused by RX frame
loss -- and **rx_gate removed that source**, so the filter is now redundant
machinery adding lag. That is exactly the kind of thing only a before/after
instrument can show.

**3. Path delay spread is 3988 ns** (14022..18010), which now dominates the
error budget. PTPv1 has no correctionField, so switch residence time lands
directly here. This is the wall, not the servo.

**4. There is a ~1.1 us standing offset** (+777..+1524 ns). Constant, so
harmless for audio — a receiver absorbs it as latency — but it means the
"sub-100 ns" target is not reachable without addressing path asymmetry.

**5. Media clock is healthy**: phase span 9 samples (0.19 ms) over the window,
ring 49..76, underrun 0/s.

## Suggested order, revised

1. ~~rx_gate~~ **done** — 21% -> 0% control-frame loss.
2. ~~Streaming telemetry~~ **done** — this file.
3. **Retune the servo, now with data.** In order of expected value:
   remove or shrink the median filter (it is costing lag for no benefit),
   then rescale KI for the measured 3.08 Hz. Verify each with a before/after
   capture rather than by ear.
4. Media clock: rate discipline is done and validated; the PHASE term is
   disabled after it limit-cycled. Redesign it as a properly damped PI with
   the integrator outside the slew limiter — and now the telemetry can show
   whether it oscillates.
5. Path delay / asymmetry is the remaining wall, and is largely inherent to
   PTPv1 on a switched network.

---

# SERVO EXPERIMENT (2026-08-04) — asked to remove the median and raise KI; measurement said no

Seven 190 s captures, one variable at a time, via `tools/telemetry.py`.

| variant | off sd | p2p | \|mean\| | rate sd | result |
|---|---|---|---|---|---|
| A median7 KI900 (original) | 79 ns | 293 | 649 ns | 0.0 ppb | baseline |
| B median1 KI900 | 171 | 1148 | 1014 | 19.5 | worse |
| C median1 KI3600 | 414 | 2753 | **26** | 43.9 | accurate, jittery |
| D median7 KI3600 | 327 | 2039 | 105 | 56.6 | worse |
| E exact integral, KI900 | 369 | 1638 | 85 | 20.6 | accurate, jittery |
| F exact + 250 ns deadband | 412 | 1769 | 134 | 22.4 | no better |
| G exact + 1000 ns deadband | — | — | — | — | **PTP UNLOCK + talker restart** |
| H = A re-run, identical code | 234 | 1084 | 737 | 9.0 | see below |

## Three things this established

**1. `rate sd = 0.0` in the baseline gave away a latent bug.** With
`SERVO_KI_DEN = 1e6`, `(-filtered * 900) / 1e6` truncates to ZERO for any
offset below **1111 ns**. The standing offset was 649 ns, under the threshold,
so the integral was frozen and the loop was structurally unable to correct it.
KI=3600 only moves that threshold to 277 ns. This is arithmetic, verifiable by
inspection, and independent of any measurement noise.

**2. The original tuning is good largely by accident.** It is not merely a
deadband -- it is a deadband PLUS severe quantisation above it (an offset of
2000 ns contributes 1 ppb). Together that makes a very low-gain, very stable
loop. Reproducing only the deadband part (G), with exact accumulation above it,
wound the integral up and drove a real PTP unlock and talker restart.

**3. THE MIDDLE OF THIS TABLE IS NOT TRUSTWORTHY.** H is byte-identical code to
A -- `git checkout` -- and measured sd 234 against A's 79. Run-to-run variance
is about 3x, which is the same size as most of the differences above. Only the
large effects are outside the noise: C/D/E's much lower standing offset, and G's
categorical failure.

**The methodological lesson, which cost this whole experiment:** a single
190 s capture per variant is not enough to rank them. The earlier claim that
"the median filter is pure lag" was worse still -- it compared raw against
filtered INSIDE one run, but the filtered value is what drives the servo, so it
shapes the raw signal too. You cannot infer the effect of removing a filter
from inside its own closed loop.

## Where this leaves it

Reverted to the original tuning: median 7, KI 900. It is the known-good
configuration and nothing measured beat it outside the noise floor.

Worth doing next, in order:
1. **Repeat captures** -- 3-5 runs per variant, interleaved, before trusting any
   ranking. The telemetry makes this cheap; not doing it is what wasted the run.
2. **Fix the truncation deliberately**: exact accumulation with the effective
   gain kept where it is today (the quantisation is currently doing the gain
   reduction by accident). That is a real robustness improvement even if the
   numbers do not move.
3. Path delay spread (3988 ns) still dominates the error budget and no servo
   change touches it.

---

# 5-RUN INTERLEAVED MATRIX (2026-08-04) — the single-capture table was noise

Redone properly: 5 variants x 5 rounds, **round-robin**, switched at RUNTIME via
a new `'s'` opcode so no variant needed a reflash. That matters — a reflash
reboots the board, which re-locks PTP, re-anchors the media clock and restarts
the talker, injecting exactly the transients a servo comparison is measuring.
25 captures, 140 s each, zero reboots.

| variant | off sd | \|mean\| standing | rate sd |
|---|---|---|---|
| A med7 ki900 | 108 [46..203] | 187 [120..535] | 0.0 [0.0..0.0] |
| B med1 ki900 | 53 [39..124] | 318 [9..764] | 0.0 [0.0..0.0] |
| C med1 ki3600 | 59 [46..157] | 36 [8..182] | 0.0 [0.0..21.7] |
| D med7 ki3600 | 92 [38..173] | 77 [22..143] | 5.0 [0.0..14.0] |
| **E med7 ki900 EXACT** | 75 [48..139] | **27 [4..90]** | 6.4 [5.2..14.6] |

(median of 5 runs, [min..max]; ns unless stated. 0 unlocks, 0 underruns in all 25.)

## Conclusions

**Offset NOISE is indistinguishable across all five.** Every range overlaps every
other. Neither the median width nor KI measurably affects jitter. The earlier
single-capture-per-variant table appeared to show 5x differences and a clear
winner; that was entirely run-to-run variance. **Both of the changes originally
requested — remove the median, raise KI — turn out to change nothing measurable
about noise.**

**The STANDING OFFSET does separate, and A vs E do not overlap at all**
([120..535] vs [4..90]). E is 7x better on the median with a tighter spread, at
the SAME gain. It simply stops discarding the remainder.

**`rate sd` 0.0 -> 6.4 ppb is the loop actually closing.** Under the truncating
form the integral was frozen for any offset below 1111 ns and could not track
real drift at all. That is the substantive fix, and it is a correctness issue,
not a tuning preference.

## Adopted

median 7, KI 900, **exact integral accumulation**. The median filter stays: no
evidence it helps, but none that it hurts either, and it is the configuration
with the most hours on it.

## Caveat worth keeping

A confirmation capture taken straight after the change read sd 692 / mean 323 /
rate sd 60.9 -- far worse than E's matrix figures. But its **path delay spread
was 6266 ns against ~4000 ns during the matrix**: the network was simply
noisier. It is not comparable, which is the same trap as before. Only
interleaved runs support a ranking.

It does show one real tradeoff: with the integral no longer frozen, rate wander
tracks network noise (60.9 ppb in that window vs 6.4 ppb in the matrix). That
is 0.06 ppm -- 8x smaller than the phase servo that was audibly bad -- and
underruns stayed at 0, but it is the price of closing the loop and is worth a
listen under load.
