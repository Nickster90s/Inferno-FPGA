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
