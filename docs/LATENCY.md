# Transmit latency: what limits us, and what 0.25 ms would take

Question: Dante receivers offer 0.25 / 0.5 / 1 / 2 / 5 ms receive latency. We
needed 2 ms. This is an FPGA with a hardware packetizer and hardware PTP
timestamping — 0.25 ms should be reachable. What is actually in the way?

Short answer: **the FPGA is not the limit.** Our per-packet emission jitter
already matches a commercial Dante transmitter. What limits us is a constant
offset error of 3.37 ms in the wrong direction, and the fact that nothing
measures or controls it correctly.

---

## 1. What a receive-latency setting actually has to cover

A receiver plays the sample stamped `T` at PTP time `T + latency`. For that to
work the packet must arrive in a window: late enough that the receiver has not
already passed `T + latency`, and early enough that it has somewhere to put it.

So the quantity that matters is, at the instant a packet hits the wire:

    lag = PTP_now - packet_timestamp        [samples, 1 sample = 20.833 us]

Positive lag = stamped in the past (normal). Negative = stamped in the future.

inferno is explicit about which side to err on (`flows_tx.rs:44`):

> it's better to have the clock in the past than in the future - otherwise
> Dante devices receiving from us go mad and fart

and puts itself at `CLOCK_OFFSET_NS = -500_000`, i.e. **24 samples in the past**.

## 2. Measured, against the PTP timeline

`tools/ts_lag.py` builds a host→PTP mapping from the PTPv1 Sync messages in the
same capture, then expresses every audio packet's arrival in PTP samples. A real
Dante device in the capture acts as the control: any systematic error in the
method applies to both streams, so the *difference* is exact.

    source           group                n     mean      min      p50      p99      max
    169.254.60.249   239.255.121.190   9992     10.8     10.1     10.8     11.7     14.0
    169.254.9.200    239.255.0.71      9992   -151.1   -152.3   -151.1   -150.7   -148.5

    169.254.60.249 - 169.254.9.200 = +161.9 samples (+3.373 ms)   <- exact

Reading these:

* **A16R (real Dante transmitter): +10.8 samples in the past.** Same side and
  same order as inferno's design value. This is the target.
* **Us: −151 samples, i.e. 3.15 ms in the FUTURE.** Exactly the case inferno
  warns about.
* **Our jitter: 3.8 samples peak-to-peak** (−152.3 … −148.5) against the A16R's
  3.9 (10.1 … 14.0). Some of that is capture timestamping, not either device.

That last line is the important one for the original question. **Our emission
timing is already as tight as a commercial device.** We do not have a jitter
problem, a fabric problem, or a scheduling problem. We have a constant in the
wrong place — and constants are free to fix.

### Caveat on the absolute numbers

The host→PTP slope fit came out at +29.4 ppm from 13 Sync messages over ~3 s,
which is implausibly large and says the absolute anchor is soft. It does not
affect the mean (evaluated at the fit centroid) and cancels entirely in the
relative figure. Design against the **+161.9 relative**, not against −151.1.

## 3. Where the 162 samples come from

The gateware stamps (`dante_packetizer.py:582,649`):

    ts_sub_emit = ts_sub + ts_offset_csr        # CSR is 0
    f_ts_sub    = ts_sub_emit - (fpp - 1)       # -15

and `ts_anchor()` loads the counter at `PTP + DANTE_TX_TS_OFFSET` with
`DANTE_TX_TS_OFFSET = 74`. With a counter that tracked PTP exactly:

    lag = 15 - 74 = -59 samples

We measure −151. **The missing 92 samples are accumulated phase drift.**
`anchors = 1`: the media clock is anchored once at boot and its phase free-runs
from then on. The NCO is rate-disciplined against PTP but never phase-corrected,
so the offset is whatever the anchor happened to catch, plus everything that has
accumulated since.

This is the "media clock is free running" observation, quantified. It also
explains the boot-to-boot variation already recorded in `mcr_dante.c:95-100`
(−37, −26, −53 on successive boots): the anchor lands somewhere different every
time and nothing pulls it back.

## 4. Why the existing `drift` statistic could not have found this

    drift_samples = emit - ptp_smp              # mcr_dante.c:319

where `emit` is the last emitted timestamp read over CSR and `ptp_smp` is PTP
read in the *main loop*. That mixes the quantity we want with main-loop
scheduling latency. It read −39 while the true on-wire figure was −151.

Two consequences, both of which cost real time:

1. It looked like we were 0.8 ms in the *past* when we were 3.15 ms in the
   *future* — the wrong side, so every inference from it pointed the wrong way.
2. **The disabled phase loop used this metric as its setpoint**
   (`PHASE_TARGET_SAMPLES (-24)`, `mcr_dante.c:377`). It was driving a broken
   measurement to inferno's number, which lands the real on-wire timestamp
   somewhere unintended. That is a candidate explanation for why enabling it
   sounded wrong, and it means "the phase loop is audible" should be retested
   against a correct metric before being treated as settled.

## 5. What to change

### A. The constant (firmware, one line)

`lag = 15 - DANTE_TX_TS_OFFSET` at zero drift, so to sit where the A16R sits:

    #define DANTE_TX_TS_OFFSET   (4)     // -> lag +11 samples, matches A16R
                                          // was 74 -> lag -59 (future)

Note which knob this is. **The anchor may take any integer**: the tick fires on
`ts_sub[0:4] == 15` and subtracts 15, so emitted timestamps are multiples of
`fpp` by construction whatever the counter was loaded with. The `ts_offset` CSR
is different — it is added *after* the tick, so it only preserves `fpp`
alignment in multiples of 16, and it has no carry into seconds (hence pinned at
0, see `dante_tx.c:204-221`). `dante_tx.h:20-23` states the multiple-of-fpp rule
without distinguishing the two paths; it is true of the CSR and false of the
anchor.

### B. Measure phase error in hardware, not in the main loop (gateware)

`DantePacketizer.__init__` already takes `tsu` and does not use it. At
`send_req`, latch

    phase_err = (tsu.seconds * 48000 + tsu.nanoseconds * 3 / 62500) - f_ts_sub

into a signed CSR. Latched at the instant of emission, this has **zero software
latency and zero scheduling jitter** — it is the true on-wire lag, available to
firmware every packet. Cost is one subtract and a register; the multiply by
48000 can be avoided by comparing against the media counter directly, since the
counter is what the anchor ties to PTP.

This is the part that genuinely wants an FPGA. A CPU cannot measure its own
emission instant to a sample; the fabric gets it for free.

### C. Hold the phase (firmware, once B exists)

With a correct error signal, options in increasing order of intrusiveness:

1. **Re-anchor on a wide band.** Cheap, but a counter reload is a timestamp
   discontinuity — one packet advances by other than `fpp`. Fine at boot, risky
   mid-stream.
2. **Slow rate trim** — the existing phase loop, retuned against the real
   metric. Pitch effect is negligible (2 ppm ≈ 0.003 cents); the historical
   objection was audible artefacts, which need re-testing against B.
3. **Signed `ts_offset` with proper carry** (gateware): make the CSR add carry
   into seconds and accept negative values, giving a glitch-free trim in steps
   of 16 samples (0.33 ms) that never touches the sample clock, the ring, or the
   USB feedback servo. Coarse, but completely free of audio-path risk.

## 6. Budget for 0.25 ms

0.25 ms = **12 samples**. Allocating:

| term | samples | note |
|---|---|---|
| target mean lag | ~5 | far enough past to never cross into the future |
| our emission jitter | ±2 | measured, already achieved |
| switch store-and-forward | 0.6 | 151 B frame at 100 Mbit; 0.06 at 1 Gbit |
| phase-control residual | ±2 | requires B + C; currently unbounded |
| **worst case** | **~10** | fits 12, with little margin |

So 0.25 ms is **plausible but tight**, 0.5 ms is comfortable, and 1 ms is easy —
*provided* phase is actually controlled. Without C the offset walks without
bound and no latency setting is safe indefinitely.

One honest caveat: the A16R's own worst-case in this capture is 14.0 samples =
0.29 ms, which would not fit 0.25 ms by this metric either. Either the metric
includes transit the receiver does not see, or 0.25 ms is intended for a
receiver closer to the source than our capture point. Do not treat 0.25 ms as
proven until it is measured end to end on a receiver that reports it.

## 7. Order of work

1. **A** alone — one constant, one flash. Should move us from −151 to about +11
   and let 1 ms work. Verify with `tools/ts_lag.py`, not with `drift`.
2. **B** — gateware, ~20 min build. Gives an honest metric and makes everything
   after it measurable.
3. **C** — only after B, and re-test the "phase loop is audible" conclusion
   against the corrected setpoint before accepting or rejecting it.

Do not skip to C. Every previous attempt at phase control was tuned against the
metric in §4.

---

# 8. How Dante latency actually works (from inferno's implementation)

## The whole rule, in one line

`flows_rx.rs:122`, the receiver:

    let latency = wrapped_diff(now, timestamp).clamp(0, i32::MAX);
    sd.actual_latency_samples.fetch_max(latency, Ordering::Relaxed);

**actual latency = `now - timestamp` at the moment of receipt, tracked as a
running MAX.** That is the number Dante Controller shows as "Latency Status",
and red means it exceeded the configured setting. There is nothing else to it.

The receiver then writes the samples at ring position
`timestamp + latency - start_time` (`flows_rx.rs:230`), i.e. the sample stamped
`T` is played at media time `T + latency`. So the requirement on a transmitter is
exactly:

    0 <= (now - timestamp) < latency        [at the receiver, on its media clock]

Two consequences worth stating plainly:

* It is a **band, not a target**. At 1 ms the band is 48 samples wide. Our
  measured per-packet spread is under 4. We have been tuning for precision that
  the protocol does not ask for.
* Below 0 the `.clamp(0, ..)` hides it: a timestamp in the FUTURE reports as
  zero actual latency, so it does not show up as a latency violation at all --
  it shows up as the receiver misbehaving in other ways, which is what
  flows_tx.rs:44 ("go mad and fart") is warning about.

## Why inferno never drifts, and we do

inferno's transmit loop (`flows_tx.rs:137-152`):

    let lag = wrapped_diff(now, flow.next_ts);
    if lag > max_lag_samples             { flow.bootstrap_next_ts(now); }   // behind
    if lag < -DISCONTINUITY_THRESHOLD    { flow.bootstrap_next_ts(now); }   // jumped
    while wrapped_diff(now, flow.next_ts) >= 0 {
        packet_ts = flow.next_ts + clock_offset_samples;   // -24
        ...
        flow.next_ts += fpp;
    }

`now` is re-read from the PTP-derived media clock on **every pass**. `next_ts` is
only a continuity counter; the PTP clock is the authority. So:

    now - next_ts is in [0, fpp)  by construction
    stamped lag  is in [24, 24 + fpp)

and it **cannot drift**, because any accumulated error is absorbed by the gate on
the next iteration. `bootstrap_next_ts` re-aligns to the next fpp boundary from
`now` whenever the error leaves the band -- inferno's own re-anchor, and it logs
"dropout occurs!" when it fires.

**Our architecture inverts this.** The hardware counter IS our timestamp source;
PTP only trims the NCO's RATE. Nothing anywhere compares the counter's PHASE
against PTP at emission. So the lag is (whatever the boot anchor caught) plus
(everything accumulated since), with no mechanism pulling it back.

That is the entire difference. Not the NCO, not the fractional increment, not
the USB servo, not fpp, not the flow table.

## So: are we missing something, or overcomplicating?

Overcomplicating. Against inferno's ~8 lines we have built a rate-disciplined
NCO, a boot anchor, a separate phase PI loop (disabled for being audible), a
fractional-increment experiment (reverted), a drift statistic that measures the
wrong thing, and a hand-calibrated offset constant. inferno needs none of it
because it re-reads PTP in the send path.

**The simple fix is inferno's, and every CSR it needs already exists.** Firmware
already anchors via `ts_load_sec/ts_load_sub/ts_load`. All that is missing is the
periodic comparison:

    if |lag - setpoint| > BAND:  ts_anchor()      // == bootstrap_next_ts(now)

with BAND wide (say +/-20 samples at 1 ms). Measured drift is ~0.44 samples/min,
so this fires roughly every 45 minutes -- rare enough that the discontinuity is a
non-event, and it is exactly what inferno does.

Do NOT reach for a phase-locked NCO. Steering the sample clock to fix a labelling
problem is what produced the audible artefacts; the label can be corrected
directly and the audio clock left alone.

## What we get that inferno cannot

inferno's stamped lag is bounded below by `clock_offset + 0` = 24 samples and
above by `24 + fpp`. At fpp=16 that is 24..40 samples (0.50..0.83 ms) -- fine for
1 ms, impossible for 0.5 or 0.25.

Measured on our hardware after calibration:

    ours  mean 11.3   spread 10.4 .. 14.2 samples   (0.22 .. 0.30 ms)
    A16R  mean 11.7   spread 10.6 .. 16.6 samples

We are already TIGHTER than a RedNet A16R, and well inside where inferno can
reach, because the packetizer paces in fabric rather than in a polling loop.
0.25 ms (12 samples) needs the mean pulled down to ~5-6 samples so the worst case
stays under 12 -- which is a change to one constant, not to the architecture.

The FPGA advantage is real. It was just being spent on a timestamp that pointed
3.4 ms into the future.
